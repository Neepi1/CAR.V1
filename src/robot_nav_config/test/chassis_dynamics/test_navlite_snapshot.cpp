#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <csignal>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include "robot_nav_config/navlite_snapshot.hpp"

using robot_nav_config::navlite_snapshot::Recorder;
namespace fs = std::filesystem;

void request_capture(const fs::path & root, const fs::path & request, const char * session = "test")
{
  nlohmann::json j{{"schema", 1}, {"session", session}, {"output_dir", root.string()},
    {"max_bytes", 1048576}, {"deadline_monotonic_ns", Recorder::monotonic_ns() + 10000000000LL}};
  std::ofstream f(request); f << j;
}
void wait_enabled(Recorder & recorder)
{
  for (int i = 0; i < 200 && !recorder.enabled(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(recorder.enabled());
}
void roundtrip(const fs::path & root)
{
  fs::create_directories(root);
  const auto request = root / "request.json";
  Recorder recorder(request.string(), root.string());
  assert(recorder.begin(1) == nullptr);  // Default off, never opens a capture.
  request_capture(root, request); wait_enabled(recorder);
  auto * frame = recorder.begin(42);
  assert(frame);
  frame->map_width = 2; frame->map_height = 2; frame->map_count = 4;
  frame->map[0] = 0; frame->map[1] = 254; frame->map[2] = 253; frame->map[3] = 255;
  frame->command = {0.3, -0.1, 0.2, 0.0, 0.0, -0.4};
  frame->command_returned = true;
  recorder.finish(frame);
  fs::remove(request);
  recorder.stop();
  bool found = false;
  for (const auto & file : fs::recursive_directory_iterator(root)) {
    if (file.path().filename() != "frames.jsonl") {continue;}
    std::ifstream f(file.path()); std::string line;
    assert(std::getline(f, line));
    const auto row = nlohmann::json::parse(line);
    assert(row.at("compute_seq") == 42);
    assert(row.at("command_returned") == true);
    assert(row.at("costmap").at("length") == 4);
    std::ifstream binary(file.path().parent_path() / "payload.bin", std::ios::binary);
    unsigned char bytes[4]{}; binary.read(reinterpret_cast<char *>(bytes), 4);
    assert(bytes[1] == 254 && bytes[3] == 255);
    found = true;
  }
  assert(found);
  std::cout << "PASS default-disabled/request/file roundtrip " << root << '\n';
}

void overflow(const fs::path & root)
{
  fs::create_directories(root); const auto request = root / "request.json";
  Recorder recorder(request.string(), root.string());
  request_capture(root, request); wait_enabled(recorder);
  const auto start = Recorder::monotonic_ns();
  for (int i = 0; i < 10000; ++i) {
    auto * f = recorder.begin(i); if (f) {recorder.finish(f);}
  }
  assert(recorder.dropped() > 0);
  assert(Recorder::monotonic_ns() - start < 1000000000LL);
  fs::remove(request);
  for (int i = 0; i < 200 && recorder.enabled(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(!recorder.enabled());
  request_capture(root, request, "clean_second_session"); wait_enabled(recorder);
  auto * second = recorder.begin(10001); assert(second); recorder.finish(second);
  recorder.stop(); recorder.stop();
  bool clean = false;
  for (const auto & file : fs::recursive_directory_iterator(root)) {
    if (file.path().filename() != "status.json") {continue;}
    nlohmann::json j; std::ifstream f(file.path()); f >> j;
    if (j.at("session") == "clean_second_session") {
      assert(j.at("dropped") == 0); assert(j.at("complete") == true); clean = true;
    }
  }
  assert(clean);
  std::cout << "PASS bounded nonblocking overflow/repeated stop dropped=" << recorder.dropped() << '\n';
}

void budget(const fs::path & root)
{
  fs::create_directories(root); const auto request = root / "request.json";
  Recorder recorder(request.string(), root.string());
  request_capture(root, request); wait_enabled(recorder);
  for (int i = 0; i < 30; ++i) {
    auto * f = recorder.begin(i);
    if (f) {f->map_count = 65536; f->map_width = f->map_height = 256; recorder.finish(f);}
    std::this_thread::sleep_for(std::chrono::milliseconds(8));
  }
  for (int i = 0; i < 100 && recorder.enabled(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(!recorder.enabled()); recorder.stop();
  bool found = false;
  for (const auto & file : fs::recursive_directory_iterator(root)) {
    if (file.path().filename() != "status.json") {continue;}
    nlohmann::json j; std::ifstream f(file.path()); f >> j;
    assert(j.at("reason") == "disk_budget"); assert(j.at("complete") == false); found = true;
    uint64_t size = 0;
    for (const auto & item : fs::directory_iterator(file.path().parent_path())) {size += item.file_size();}
    assert(size <= 1048576);
  }
  assert(found); std::cout << "PASS disk budget leaves partial evidence\n";
}

void write_error(const fs::path & root)
{
  const auto child = fork(); assert(child >= 0);
  if (child == 0) {
    fs::create_directories(root); const auto request = root / "request.json";
    Recorder recorder(request.string(), root.string());
    request_capture(root, request); wait_enabled(recorder);
    std::signal(SIGXFSZ, SIG_IGN);
    struct rlimit limit{4096, 4096}; assert(setrlimit(RLIMIT_FSIZE, &limit) == 0);
    auto * f = recorder.begin(7); assert(f); f->map_count = 65536;
    recorder.finish(f);
    for (int i = 0; i < 100 && recorder.enabled(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    assert(!recorder.enabled()); recorder.stop(); _exit(0);
  }
  int status = 0; assert(waitpid(child, &status, 0) == child);
  assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  std::cout << "PASS writer disk failure never escapes producer or deadlocks stop\n";
}

void throughput(const fs::path & root)
{
  fs::create_directories(root); const auto request = root / "request.json";
  Recorder recorder(request.string(),root.string());
  auto j=nlohmann::json{{"schema",1},{"session","performance"},{"output_dir",root.string()},
    {"max_bytes",16777216},{"deadline_monotonic_ns",Recorder::monotonic_ns()+10000000000LL}};
  {std::ofstream f(request); f<<j;} wait_enabled(recorder);
  struct rusage before{},after{}; getrusage(RUSAGE_SELF,&before);
  const auto started=Recorder::monotonic_ns(); std::vector<int64_t> copy_ns;
  std::array<unsigned char,40000> grid{};
  for(unsigned i=0;i<30;++i) {
    const auto start=Recorder::monotonic_ns(); auto * frame=recorder.begin(i); assert(frame);
    frame->map_count=40000; frame->map_width=frame->map_height=200;
    std::memcpy(frame->map.data(),grid.data(),grid.size());
    frame->sequence_count=48; frame->sequence_original_count=48;
    recorder.finish(frame); copy_ns.push_back(Recorder::monotonic_ns()-start);
    std::this_thread::sleep_for(std::chrono::milliseconds(67));
  }
  recorder.stop(); getrusage(RUSAGE_SELF,&after); assert(recorder.dropped()==0);
  std::sort(copy_ns.begin(),copy_ns.end());
  const auto cpu=[](const rusage & r) {return double(r.ru_utime.tv_sec+r.ru_stime.tv_sec)+
      double(r.ru_utime.tv_usec+r.ru_stime.tv_usec)/1e6;};
  const double seconds=(Recorder::monotonic_ns()-started)/1e9;
  std::cout<<"PERF synthetic40KB_15Hz producer_p50_us="<<copy_ns[15]/1e3
    <<" producer_max_us="<<copy_ns.back()/1e3<<" recorder_process_cpu_percent_one_core="
    <<100*(cpu(after)-cpu(before))/seconds<<" preallocated_queue_bytes="
    <<sizeof(robot_nav_config::navlite_snapshot::Frame)*robot_nav_config::navlite_snapshot::kSlots
    <<" dropped=0 (not a production performance claim)\n";
}

int main()
{
  const auto root = fs::temp_directory_path() / ("navlite_snapshot_test_" + std::to_string(getpid()));
  roundtrip(root / "roundtrip"); overflow(root / "overflow");
  budget(root / "budget"); write_error(root / "write_error");
  throughput(root / "throughput");
  {Recorder recorder((root / "missing").string(), root.string()); recorder.stop(); recorder.stop();}
  std::cout << "PASS immediate idle stop\n";
}
