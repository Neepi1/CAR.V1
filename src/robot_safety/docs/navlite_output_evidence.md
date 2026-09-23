# Lightweight safety output evidence

2026-09-20 diagnostic candidate only. No arbitration, BMS contact/latch,
reverse/lateral permission, motion parameter, ROS endpoint, thread, or lock change.

## Event contract

- `NAVLITE safety event=diagnostics_ready schema=2`: one WARN startup line identifies the
  configured normal/API/docking/output topics and whether the pre-existing
  wheel-odometry subscription is available. It is not proof that messages arrived.
- `NAVLITE safety`: emitted after the unchanged final publish, with actual
  input when available, selected source, final output and interception reason.
  `output_seq` counts this process's final publications, not a DDS sequence or
  proof of delivery to Ranger. Normal continuous output is not logged per frame.
- `NAVLITE safety_state ... publication=state_only`: a state release is not
  a published nonzero command or evidence that the vehicle moved.
- `NAVLITE safety_arbitration ... publication=no_message`: that input lost
  source arbitration; this is not a published zero.

State/reason changes are immediate. The same ongoing zero/interception is
limited to one log per steady-clock second; the existing callback and timer
publications for one cause share the log key. No global log level increase.

The final output event reuses the existing wheel-odometry callback's latest
signed vx/vy/wz, source header stamp/frame/child, and steady receive age. When
that subscription is disabled or unseen, `wheel_known=0` and unknown numerical
values are written. `actual_mode_rx_age_sec` is the age of the last accepted
existing mode-status observation, not raw CAN age. These samples can precede
the command being logged: they are **not a measured response to that command**.
The source ROS stamp and steady receipt age are different time domains; do not
subtract them to claim latency. There is no continuous wheel/command recording.

## Build provenance and tests

The active safety executable and the BMS-current-scope build manifest match
SHA256 `58bbc9ff94de6d96bb5a33447e212ad568c5a58d6366f936d7b5230ec9f9b60a`.
Its complete original object/archive link closure was relinked unchanged first
and reproduced that hash. Only `robot_safety_node.cpp` is newly compiled for
the diagnostic candidate; the deployed BMS/elevator policy archives are reused.
The candidate/source manifest and exact deployed-source diagnostic diff are in
`/tmp/njrh_reports/navlite_safety_20260920_jPCam2` on Jetson.

`test/test_navlite_output_isolated.py` starts the real safety executable and
compares actual final Twist output with actual file logs. It requires an
explicit private network namespace, domain 183, private IPC/SHM, a report-only
binary copy, and has no chassis process. It must not be run in the robot domain.

Two evidence tests first fail against the byte-identical deployed binary (no
schema/wheel event). The diagnostic candidate passes all eight output/file-tail tests:
startup schema; upstream zero/resume; estop state-only release; watchdog versus
explicit zero; docking source priority; repeated reverse denial rate limit;
reuse of the existing wheel callback; real node stdout to file to recorder.
The final candidate also passes the 25 BMS contact/current-scope regressions.
This validates synthetic events, not
physical navigation, charging, stop distance, scheduler load or production log
landing. BMS protection regressions and recorder ingestion results are kept in
the report, not inferred from a compile success. A six-second healthy-stream
comparison observed 16 process CPU ticks in both baseline and candidate, about
2.66% of one core, and zero added steady-state log bytes. This short isolated
measurement is not a real-car load or worst-case timing guarantee.

Future activation requires separately authorized incremental deployment and
restart. Confirm schema and real events land in `robot_safety_common.log` before
asking for a new user-controlled avoidance capture. No deployment is included
in this source/test change.
