#ifdef NDEBUG
#undef NDEBUG
#endif
#include "robot_bringup/runtime_flatscan_snapshot.hpp"
#include <cassert>
#include <iostream>

using namespace robot_bringup::flatscan;
rapidjson::Document fixture() {
  rapidjson::Document d;
  d.Parse(R"({"schema":"njrh.runtime_health.v1","implementation":"cpp",
    "boot_id":"test-boot\n","generation":"guard-1","sequence":100,
    "updated_at":1000,"updated_monotonic_sec":100,"clock_valid":true,"sampling_delayed":false,
    "flatscan_monitor":{"schema":"njrh.flatscan_monitor.v1","metadata_valid":true,
      "metadata_max_age_sec":3,"graph_max_age_sec":15,
      "input":{"schema":"njrh.flatscan_input.v1","boot_id":"test-boot","generation":"localizer-1",
        "sequence":11,"input_supported":true,"available":true,"input_sequence":300,
        "received_monotonic_sec":99.8,"emitted_monotonic_sec":100,"header_stamp_sec":50},
      "stamp_progress":{"generation":"localizer-1","seen_advancing":true,"replay_detected":false,
        "header_watermark_sec":50,"last_advance_monotonic_sec":99.8,"last_advance_input_sequence":300},
      "graph":{"valid":true,"sequence":20,"checked_monotonic_sec":99,
        "scan_publishers":1,"flatscan_publishers":1,"metadata_publishers":1}}})");
  assert(!d.HasParseError());
  return d;
}
Decision check(const Json &d, double now = 100.1, double wall = 1000.1) {
  return evaluate(d, now, wall, "test-boot\n", 2, 10);
}
int main() {
  auto d = fixture(); assert(check(d).code == 0);
  assert(check(d).source_id == "guard-1:localizer-1");
  assert(check(d).evidence_sequence == 11);
  assert(check(d, 103, 1003).code == 40);
  assert(check(d, 99, 999).code == 40);
  assert(check(d, 100.1, 1004).code == 40);
  for (const char *key : {"generation", "sequence", "boot_id", "clock_valid", "sampling_delayed",
      "updated_monotonic_sec", "updated_at", "flatscan_monitor"}) {
    auto bad = fixture(); bad.RemoveMember(key); assert(check(bad).code == 40);
  }
  d = fixture(); d["boot_id"].SetString("other-boot"); assert(check(d).code == 40);
  d = fixture(); d["clock_valid"].SetBool(false); assert(check(d).code == 40);
  d = fixture(); d["sampling_delayed"].SetBool(true); assert(check(d).code == 40);
  for (const char *key : {"generation", "sequence", "boot_id", "available", "input_sequence",
      "emitted_monotonic_sec", "received_monotonic_sec", "input_supported", "header_stamp_sec"}) {
    auto bad = fixture(); bad["flatscan_monitor"]["input"].RemoveMember(key);
    assert(check(bad).code == 40);
  }
  d = fixture(); d["flatscan_monitor"]["input"]["available"].SetBool(false); assert(check(d).code == 40);
  d = fixture(); d["flatscan_monitor"]["input"]["input_supported"].SetBool(false); assert(check(d).code == 40);
  d = fixture(); d["flatscan_monitor"]["input"]["emitted_monotonic_sec"].SetDouble(95); assert(check(d).code == 40);
  d = fixture(); d["flatscan_monitor"]["input"]["received_monotonic_sec"].SetDouble(102); assert(check(d).code == 40);
  for (const int n : {0, 2, -1}) {
    d = fixture(); d["flatscan_monitor"]["graph"]["metadata_publishers"].SetInt(n); assert(check(d).code == 40);
  }
  d = fixture(); d["flatscan_monitor"]["graph"]["valid"].SetBool(false); assert(check(d).code == 40);
  d = fixture(); d["flatscan_monitor"]["graph"]["checked_monotonic_sec"].SetDouble(80); assert(check(d).code == 40);
  d = fixture(); d["flatscan_monitor"]["graph"]["flatscan_publishers"].SetInt(0);
  assert(check(d).code == 0);  // Received data outranks cached graph absence.
  d["flatscan_monitor"]["input"]["received_monotonic_sec"].SetDouble(80);
  d["flatscan_monitor"]["stamp_progress"]["last_advance_monotonic_sec"].SetDouble(80);
  assert(check(d).code == 50 && check(d).evidence_kind == "graph");
  assert(check(d).evidence_sequence == 20);
  d["sequence"].SetUint64(999);
  assert(check(d).evidence_sequence == 20);  // Guard rewrites are not new graph evidence.
  d["flatscan_monitor"]["graph"]["flatscan_publishers"].SetInt(1);
  assert(check(d).code == 50 && check(d).evidence_kind == "input");
  assert(check(d).evidence_sequence == 11);
  d["flatscan_monitor"]["metadata_valid"].SetBool(false); assert(check(d).code == 40);
  d = fixture(); d["flatscan_monitor"]["input"]["header_stamp_sec"].SetDouble(0);
  d["flatscan_monitor"]["stamp_progress"]["header_watermark_sec"].SetDouble(0);
  assert(check(d).code == 40);
  d = fixture(); d["flatscan_monitor"].RemoveMember("stamp_progress"); assert(check(d).code == 40);

  StampProgress progress;
  auto observed = fixture();
  auto &input = observed["flatscan_monitor"]["input"];
  auto update = [&] {
    progress.observe(input);
    observed["flatscan_monitor"]["stamp_progress"] = progress.snapshot(observed.GetAllocator());
  };
  update(); assert(check(observed).code == 40);  // First sample is only a baseline.
  input["input_sequence"].SetUint64(301); input["header_stamp_sec"].SetDouble(51);
  update(); assert(check(observed).code == 0);
  const double advance = number(field(observed["flatscan_monitor"]["stamp_progress"], "last_advance_monotonic_sec"));
  input["sequence"].SetUint64(12);  // Heartbeat alone must not renew stamp advancement.
  update(); assert(check(observed).code == 0);
  assert(number(field(observed["flatscan_monitor"]["stamp_progress"], "last_advance_monotonic_sec")) == advance);
  input["input_sequence"].SetUint64(302); input["received_monotonic_sec"].SetDouble(99.9);
  update(); assert(check(observed).code == 40);  // New receives, same old frame stamp.
  assert(boolean(field(observed["flatscan_monitor"]["stamp_progress"], "replay_detected")));
  assert(number(field(observed["flatscan_monitor"]["stamp_progress"], "last_advance_monotonic_sec")) == advance);
  input["input_sequence"].SetUint64(303); input["header_stamp_sec"].SetDouble(49);
  update(); assert(check(observed).code == 40);  // Regressed header cannot lower watermark.
  input["input_sequence"].SetUint64(304); input["header_stamp_sec"].SetDouble(50);
  update(); assert(check(observed).code == 40);
  input["input_sequence"].SetUint64(305); input["header_stamp_sec"].SetDouble(52);
  update(); assert(check(observed).code == 0);
  input["generation"].SetString("localizer-2");
  update(); assert(check(observed).code == 40);  // New source must establish advancement again.
  input["input_sequence"].SetUint64(306); input["header_stamp_sec"].SetDouble(53);
  update(); assert(check(observed).code == 0);
  input["input_sequence"].SetUint64(307); input["header_stamp_sec"].SetDouble(0);
  update(); assert(check(observed).code == 40);
  std::cout << "FlatScan snapshot policy cases passed\n";
}
