# Return-to-Dock Rotation Trace

## Commercial intent

The current docking flow has a high completion rate. This recorder does not
merge, remove, or retune any yaw stage. It preserves the Nav2 stop-and-handoff
barrier, safety arbitration, bridge settle gate, predock validation, and the
fine-docking contact controller. Its only purpose is to identify which command
owner produced each visible rotation and why zero velocity existed between
them.

The recorder is read-only:

- one ROS 2 participant;
- no velocity, goal, parameter, or service publisher/client;
- authenticated HTTP `GET` only;
- no `/scan`, PointCloud2, rosbag, or TF subscription;
- bounded duration and clean `Ctrl+C` finalization;
- report output only below `/tmp/njrh_reports`.

## Ranked hypotheses and falsifiable predictions

1. **Normal Nav2-to-docking handoff exposes two yaw tolerances.**
   The first physical rotation will overlap `/cmd_vel_nav_raw`; after a zero
   handoff window, the second will overlap `/cmd_vel_docking` while
   `predock_yaw_align_active=true` or a fine-docking phase is active.
2. **Collision or safety gating splits one intended rotation.**
   An upstream yaw command will remain nonzero during the visible stop while
   `/cmd_vel_collision_checked` or final `/cmd_vel` becomes zero, with a
   matching collision/safety state change.
3. **Ranger motion-mode switching or spin settle creates the stop.**
   The pause will overlap `mode_changing=true`, a change in actual Ranger mode,
   or local-state spin-settle evidence even though a yaw stage remains active.
4. **The bridge/fine-entry gate creates an intentional zero window.**
   All command sources will be zero while the API reports
   `FINE_DOCKING_BRIDGE_SETTLE` or an incomplete fine-entry gate.
5. **Nav2 itself restarts yaw after a controller/path event.**
   Both visible rotations will originate on `/cmd_vel_nav_raw`, with no
   intervening `/cmd_vel_docking` yaw command.

The script's red-capable verdict is
`RED_ROTATE_STOP_ROTATE_CAPTURED`: wheel odometry contains two physical yaw
episodes separated by at least 0.45 seconds and no more than 45 seconds. A
`NO_TWO_STAGE_ROTATION_CAPTURED` result is not evidence for changing control
parameters; it only means that run did not reproduce the symptom.

## Run from the operator computer

Start the recorder first:

```powershell
ssh -t nvidia@192.168.31.23 "docker exec -it NJRH-car bash --noprofile --norc /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/record_docking_rotation_trace.sh --duration-sec 240 --label return_dock_01"
```

Wait until it prints `READY`. Then use the existing App/business flow to start
return-to-dock. The recorder does not initiate motion. After the symptom or
successful docking, press `Ctrl+C`; this finalizes the report immediately.
Otherwise it exits automatically after the configured duration.

The final terminal lines print the exact report directory, for example:

```text
/tmp/njrh_reports/docking_rotation_trace_YYYYMMDDTHHMMSSZ_return_dock_01
```

To inspect the concise result over SSH:

```powershell
ssh nvidia@192.168.31.23 "sed -n '1,220p' /tmp/njrh_reports/docking_rotation_trace_YYYYMMDDTHHMMSSZ_return_dock_01/summary.md"
```

To copy the entire trace back:

```powershell
scp -r nvidia@192.168.31.23:/tmp/njrh_reports/docking_rotation_trace_YYYYMMDDTHHMMSSZ_return_dock_01 .
```

## Evidence files

- `summary.md`: physical rotation episodes and stop/handoff classification.
- `analysis.json`: machine-readable verdict and episode windows.
- `command_events.csv`: every low-bandwidth velocity boundary message.
- `timeline.csv`: time-aligned command, odometry, mode, safety, and API state.
- `api_samples.csv`: docking/navigation phase, initial/final yaw errors, trigger
  result, bridge settle, and fine-entry fields.
- `state_events.jsonl`: only state transitions, not high-rate raw status dumps.
- `graph.json`: command publishers observed after DDS warmup and the recorder's
  owned endpoint inventory.
- `logs_since_start/`: filtered runtime log lines generated during the capture.
- `SHA256SUMS`: report integrity manifest.

## Interpretation boundary

Only consider hysteresis or a stable-sample skip when the trace repeatedly
shows that:

1. the second episode is on `/cmd_vel_docking`;
2. its initial yaw error was already continuously inside the predock
   tolerance;
3. no bridge, fine-entry, Ranger-mode, collision, or safety gate required the
   stop;
4. docking completion and contact braking remain unaffected.

Do not apply the elevator motion controller to the contact phase and do not
remove the zero-speed control handoff.

## Real-hardware validation still required

- Capture at least one smooth docking and one visibly segmented docking using
  the same saved dock and floor.
- Note the report directory and the operator-observed approximate stop time.
- Confirm `graph.json` reports zero recorder-owned publishers.
- Compare the episode sources, yaw initial/final errors, Ranger mode, and
  safety/collision evidence before proposing any controller change.
