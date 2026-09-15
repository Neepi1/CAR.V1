// Pure C++ checks: no ROS context or network participant is created.
#define STARTUP_CONTEXT_OBSERVER_CORE_ONLY
#include "../src/startup_context_observer.cpp"

#include <cassert>
#include <iostream>

std::string status(const std::string & sequence, const std::string & has_map = "true",
  const std::string & owner = "robot_localization_bridge")
{
  return "{\"last_explicit_relocalization_sequence\":" + sequence +
         ",\"has_map_to_odom\":" + has_map +
         ",\"map_to_odom_publisher_owner\":\"" + owner + "\"}";
}

template<class Operation>
void rejects(Operation operation)
{
  bool rejected = false;
  try {operation();} catch (const std::exception &) {rejected = true;}
  assert(rejected);
}

int main()
{
  using namespace startup_context;
  using startup_context::source_is_current;
  assert(!source_is_current(99, 100, 110));
  assert(source_is_current(100, 100, 110));
  assert(source_is_current(110, 100, 110));
  assert(!source_is_current(111, 100, 110));
  assert(!source_is_current(0, 100, 110));
  assert(!source_is_current(-1, 100, 110));
  assert(!source_is_current(100, 0, 110));
  const std::string valid = R"({"last_explicit_relocalization_sequence":"7","has_map_to_odom":"true","map_to_odom_publisher_owner":"robot_localization_bridge"})";
  assert(startup_context::bridge_sequence(valid, "6") == "7");
  assert(!startup_context::bridge_sequence(valid, "7"));
  assert(!startup_context::bridge_sequence("bad JSON", "6"));
  assert(bridge_sequence(status("7"), "6") == "7");
  assert(bridge_sequence(status("0"), "-1") == "0");
  assert(bridge_sequence(status("\"18446744073709551616000\""), "7") ==
    "18446744073709551616000");
  assert(bridge_sequence(status("18446744073709551616000"), "7") ==
    "18446744073709551616000");
  for (const auto & sequence : {"true", "false", "7.0", "7e0", "-1", "null",
    "[]", "{}", "\"07\"", "\"+7\"", "\"7.0\"", "\" 7\""})
  {
    assert(!bridge_sequence(status(sequence), "6"));
  }
  assert(!bridge_sequence(status("6"), "6"));
  assert(!bridge_sequence(status("7", "false"), "6"));
  assert(!bridge_sequence(status("7", "1"), "6"));
  assert(!bridge_sequence(status("7", "true", "wrong_owner"), "6"));
  assert(!bridge_sequence("[]", "6"));
  assert(!bridge_sequence("{}", "6"));
  const auto quoted = R"("{\"last_explicit_relocalization_sequence\":7,\"has_map_to_odom\":true,\"map_to_odom_publisher_owner\":\"robot_localization_bridge\"}")";
  assert(bridge_sequence(quoted, "6") == "7");
  assert(!bridge_sequence(R"({"last_explicit_relocalization_sequence":7,"has_map_to_odom":true,"has_map_to_odom":false,"map_to_odom_publisher_owner":"robot_localization_bridge"})", "6"));

  // Prewarm callbacks and 20 samples queued while stopped must NOT provide
  // proof. Arrival at now=300 cannot disguise old DDS source stamps below 200.
  Evidence evidence("6");
  evidence.observe(valid, 150, 150);
  assert(!evidence.sequence());
  evidence.begin_commit(100, 200);
  for (int index = 0; index < 20; ++index) {
    evidence.observe(valid, 180 + index, 300);
    assert(!evidence.sequence());
  }
  evidence.observe(valid, 301, 300);
  evidence.observe(valid, 0, 300);
  assert(!evidence.sequence());
  evidence.observe(valid, 200, 300);
  assert(evidence.sequence() == "7");
  evidence.begin_commit(400, 350);
  assert(!evidence.sequence());
  evidence.observe(valid, 399, 500);
  assert(!evidence.sequence());
  evidence.observe(valid, 400, 500);
  assert(evidence.sequence() == "7");

  assert(commit_timestamp("100\n", 100) == 100);
  assert(commit_timestamp(" \t99\r\n", 100) == 99);
  for (const auto & invalid : {"", " ", "0", "01", "-1", "+1", "100.0", "1e2",
    "1 00", "101", "9223372036854775808", "100\n100"})
  {
    assert(!commit_timestamp(invalid, 100));
  }
  assert(handoff_record_pending("bad JSON"));
  assert(handoff_record_pending("[]"));
  assert(handoff_record_pending("{}"));
  for (const auto & version : {"1", "1.0", "1e0", "true"}) {
    for (const auto & terminal : {"committed", "failed"}) {
      assert(!handoff_record_pending(std::string("{\"schema\":\"njrh.floor_startup_handoff.v1\",\"version\":") +
        version + ",\"state\":\"" + terminal + "\"}"));
    }
  }
  assert(handoff_record_pending(R"({"schema":"njrh.floor_startup_handoff.v1","version":"1","state":"committed"})"));
  assert(handoff_record_pending(R"({"schema":"njrh.floor_startup_handoff.v1","version":1,"state":"requested"})"));
  assert(handoff_record_pending(R"({"schema":"wrong","version":1,"state":"committed"})"));
  assert(handoff_record_pending(R"({"schema":"njrh.floor_startup_handoff.v1","version":1,"state":"committed","state":"requested"})"));

  auto options = parse_options({"observer", "--minimum-sequence", "6"});
  assert(options.warmup_wait_sec == 0 && options.service_wait_sec == 10 &&
    options.bridge_wait_sec == 15 && options.commit_file.empty());
  options = parse_options({"observer", "--minimum-sequence", "-1", "--commit-file", "/tmp/commit",
      "--warmup-wait-sec", "3.5", "--service-wait-sec", "1", "--bridge-wait-sec", "2"});
  assert(options.minimum_sequence == "-1" && options.commit_file == "/tmp/commit" &&
    options.warmup_wait_sec == 3.5 && options.service_wait_sec == 1 && options.bridge_wait_sec == 2);
  rejects([] {parse_options({"observer"});});
  rejects([] {parse_options({"observer", "--minimum-sequence", "-2"});});
  rejects([] {parse_options({"observer", "--minimum-sequence", "6", "--warmup-wait-sec", "1"});});
  rejects([] {parse_options({"observer", "--minimum-sequence", "6", "--commit-file", ""});});
  rejects([] {parse_options({"observer", "--minimum-sequence", "6", "--unknown", "1"});});
  rejects([] {parse_options({"observer", "--minimum-sequence"});});
  for (const auto & bad : {"-1", "nan", "inf", "1s", ""}) {
    rejects([&] {parse_options({"observer", "--minimum-sequence", "6", "--service-wait-sec", bad});});
  }
  std::cout << "startup context source/JSON/commit/handoff/options checks passed\n";
}
