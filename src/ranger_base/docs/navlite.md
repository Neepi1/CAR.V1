# NAVLITE chassis evidence

This is diagnostic instrumentation, not a controller, watchdog, mode policy or
CAN monitor. No command value, transition predicate, timeout or ROS interface
changes. No extra subscription, CAN request, thread or lock is introduced.

The existing messenger thread saves scalar copies at command receipt, after an
existing `SetMotionCommand()` returns, and from the existing SDK state reads and
published odometry. `NAVLITE chassis schema=1` uses WARN so the current logging
threshold does not hide it. A one-time `NAVLITE chassis event=diagnostics_ready` announces
coverage. It should land in `ranger_chassis_common.log`; production log landing
must be verified after a separately authorized deployment/restart.

## Evidence semantics

- `input_*`, `in=(vx,vy,wz)`: final `/cmd_vel` as received before the existing
  lateral deadband. `input_seq` is this process's callback count, not a publisher
  or CAN sequence. Twist has no source header: `input_source_stamp=unknown`.
- `sdk_linear/steer/angular`: the actual arguments passed to the SDK, not a
  body-frame Twist. `sdk_submit_seq` advances only after that existing call;
  `can_tx_confirmed=unknown` because asynchronous CAN enqueue is not confirmation
  of transmission or execution. Side-slip's third SDK argument is preserved
  verbatim and must not be mistaken for physical angular velocity.
- `reason=mode_switch_hold` distinguishes an internally submitted zero from a
  received zero (`reason=motion_command`). `callback_submitted` describes the
  last callback, not a new command caused by the diagnostic event.
- `input_gap=1` means no callback for more than the diagnostic-only 0.5-second
  threshold. It does not assert why messages stopped, does not imply a zero
  was sent, and changes no watchdog or control behavior.
- Actual/desired mode, `mode_changing`, the existing handshake state, elapsed
  time and stop-stable flag are observations of existing variables.
- `raw_linear/steer/angular`, four `wheel_speed` and four `wheel_angle` values
  are SDK cached feedback. `feedback_read_seq/age` describe reading that cache,
  not fresh CAN frames. Values retain the SDK's units (motion m/s/rad/s/rad,
  wheel motor speeds and angles as decoded by its protocol).
- `core_group_stamp_ns/age_sec` are SDK steady-clock receipt times shared by
  system/motion/light/mode/RC frames. **Another type can refresh this timestamp
  without refreshing motion.** `actuator_group_stamp` is even broader: the SDK
  refreshes it for each decoded CAN frame before dispatch. Neither proves wheel
  or motion freshness. Per-motion/per-wheel CAN age and sequence explicitly
  remain `unknown`; the SDK is not changed for this instrumentation.
- `odom_*` / `actual=(vx,vy,wz)` are the existing derived wheel-odom cache.
  `odom_stamp_kind=driver_publish_time` is explicit: the header uses the driver's
  timer clock and is not a CAN measurement timestamp. `motion_fresh=unknown`
  prevents a timer-refresh from masquerading as fresh physical feedback.
- `publication=observation` means the log itself is not a velocity publication.

All ages measured by this helper use `steady_clock`. The odom ROS stamp is kept
separately; do not subtract it from the monotonic or receive time. Missing ages
are `nan`; missing source stamps/sequences are `unknown` (never invented zero).

## Bounded work

Each invocation compares a bounded set of scalar fields. No formatting occurs
unless an event is retained. Command/SDK sign classification uses a 0.002/0.005
hysteresis **only for log selection**, retaining raw signed values and explicit
exact-zero fields. Mode/reason/sign/gap transitions log immediately; otherwise
active observations are at most 1 Hz. Idle after an input gap produces no
periodic log unless a mode transition is still pending. It is not a full-rate
velocity trace, and tiny pulses entirely inside the diagnostic hysteresis are
not guaranteed to be retained. Existing control thresholds are unchanged.

## Isolated validation

`test_navlite_chassis_trace` is pure C++14: signed input/submission, unknown
timestamps, mode hold, input gap, rate limit and noise tests.

`test_navlite_messenger_fake_sdk` compiles the real messenger with test-only
RangerRobot symbol replacements. It does **not link ugv_sdk**; `Connect()` is a
fake function and no socket or CAN interface is opened. GCC access-control
override exists only on this test target. It exercises the real command
callback and feedback/odom path: forward, zero, lateral-mode hold, confirmed
release, and input gap with no synthesized SDK calls. Assertions stay enabled
in Release builds. It is not registered as an automatic CTest ROS test.

Run only in a private network/IPC/mount namespace, private `/dev/shm`, ROS domain
181, and a unique ROS_HOME under `/tmp/njrh_reports`; never in the vehicle ROS
domain. For example (inside the prepared container):

```bash
unshare --net --ipc --mount --fork bash -c '
  mount --make-rprivate /
  mount -t tmpfs tmpfs /dev/shm
  export ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 NAVLITE_ISOLATED=1
  export ROS_HOME=/tmp/njrh_reports/<candidate>/ros_home
  timeout 15 /tmp/njrh_reports/<candidate>/build/test_navlite_messenger_fake_sdk
'
```

Production output/cadence and physical motion are not validated by this test.
Unknown CAN freshness must remain visible in reports; no definitive physical
response claim may rest only on a stale cache or nonzero SDK submission.
