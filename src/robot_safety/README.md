# robot_safety

Final command arbitration point before the chassis bridge.

## Canonical Contract

- normal navigation/App command input: `/cmd_vel_collision_checked`
- post-call elevator velocity-smoother input: `/cmd_vel_nav` (inactive unless
  the exact transaction-scoped permit is valid; not raw controller output)
- elevator-entry bypass permit:
  `/ranger_mini3/elevator_entry_collision_bypass`
- docking command input: `/cmd_vel_docking`
- final chassis command output: `/cmd_vel`
- safe command mirror: `/cmd_vel_safe` (diagnostic only)
- estop input: `/safety/estop`
- optional localization gate: `/localization/health`
- owner-scoped motion hold service: `/safety/set_motion_hold`
- atomic recovery hold-release service:
  `/safety/release_motion_hold_if_execution_idle`
- legacy/general execution lease service: `/safety/set_execution_lease`
  (the elevator-test workflow does not acquire it)
- motion interlock state: `/safety/motion_interlock_state`
- state output: `/safety/status`
- motion-allowed output: `/safety/motion_allowed`

## Arbitration Policy

`robot_safety` is the only package allowed to publish the post-arbitration chassis command on `/cmd_vel`. `/cmd_vel_safe` is a read-only diagnostics mirror. Docking commands use a separate `/cmd_vel_docking` input so Nav2 `collision_monitor` zero commands on `/cmd_vel_collision_checked` cannot overwrite near-field docking or undocking commands.

The navigation exception covers every post-call elevator intent:
`SOURCE_LANDING_FACE_CABIN`, `REVERSE_ENTRY_STAGING`, `ENTER_CABIN`,
`REVERSE_ENTER_CABIN`, `CABIN_PANEL_APPROACH`, `RETURN_CABIN_CENTER`, and
`TARGET_LANDING`. The elevator runtime refreshes a
permit containing the exact active `transaction_id`; while the same
transaction owns the exact `robot_elevator_manager` operating-mode contract,
the hold is clear, and the operating mode is `ELEVATOR_WAIT` (post-call staging)
or `DOORWAY` (cabin entry/exit), `robot_safety` accepts
velocity-smoother output from
`/cmd_vel_nav` instead of the collision-monitor-checked stream. Permit expiry,
transaction/mode/lease change, command staleness, cancellation, or any
terminal result produces zero. Only the pre-call hall approach and ordinary
navigation remain on `/cmd_vel_collision_checked`. Estop,
localization health, final speed clamps, reverse/lateral permits, and command
watchdogs remain enforced in both paths.

Neither mode alone enables the exception. The pre-call hall approach sends no
bypass permit; stationary `ELEVATOR_RIDE` and ordinary `NORMAL` are rejected.
See [post-call collision scope](docs/elevator_post_call_collision_bypass.md).

Command topics are treated as latest-only control streams, not durable command queues. The node creates the normal, API, docking, final, and mirror Twist endpoints with `KEEP_LAST(1)` QoS by default so old nonzero velocity samples cannot be drained after a newer stop command. A near-zero command from the active command owner is also stop-dominant: `robot_safety` immediately publishes zero, keeps a short zero burst window, and rejects late nonzero samples during that window. This prevents spin/drive/arc commands from continuing only because an upstream queue or source-priority cache still contains older velocity samples.

The current arbitration order is:

1. `ESTOP_ACTIVE`
2. `MISSION_MOTION_HOLD` while one or more owner-scoped holds exist
3. `EXECUTION_LEASE_MISSING` after an execution session has engaged and its lease is absent or expired
4. `EXECUTION_MODE_INVALID` when the matching special-mode lease is absent,
   stale, expired, or owned by another mission
5. `EXECUTION_SOURCE_BLOCKED` when API or docking velocity attempts to preempt
   a healthy elevator execution session
6. `LOCALIZATION_INVALID` when `require_localization_health=true`
7. `DOCKED_CONTACT_BLOCK` when normal motion is requested while docked or charging
8. `COMMAND_STALE` when upstream control stops refreshing commands inside the watchdog window
9. `OK`

When a fresh `/cmd_vel_docking` message exists, normal `/cmd_vel_collision_checked` messages are ignored for `docking_cmd_priority_timeout_sec`. This keeps App zero bursts and Nav2/collision-monitor zero output from interleaving with controlled docking motion.

After a fresh actual entry into SPINNING reported by `/ranger_base/status`, `robot_safety` can hold the first following linear command at zero until the spin tail has settled. Low-speed Twist values and desired mode never prove actual spin. The gate checks `/wheel/odom.twist.twist.angular.z` and, by default, `/lidar_imu_bias_corrected.angular_velocity.z` because the Ranger wheel twist can report zero before the physical body yaw-rate has actually stopped. `/local_state/odometry` remains an optional diagnostic gate only; production release is based on raw wheel odom plus the corrected 100 Hz IMU tail detector. The local-state runtime must keep `imu_gyro_bias_filter_node` resident even when the EKF profile is wheel-only; otherwise this gate can only fail open after its timeout. This handles Ranger Mini 3 spin stop tail: upstream Nav2/API may have already published zero yaw, while the chassis is still rotating for a short interval. The gate is intentionally placed in `robot_safety` because this package is the final command arbitration point before `/cmd_vel`; it does not change Nav2, AMCL, or the chassis SDK motion model.

Repeated SPINNING feedback does not reset settling or rearm a consumed episode.
Rotation/zero commands pass without consuming the pending handoff; mode exit or
feedback loss does not erase an already observed spin tail. Release still uses
the existing wheel/IMU stability or bounded timeout, without waiting for an
Ackermann acknowledgement. Missing actual feedback does not create a new motion
gate. The existing status subscription is shared by spin settling and the mode
exit guard; disabling the latter must not disconnect the former. See
[actual-mode handoff](docs/spin_to_drive_actual_mode.md).

After docking or lateral capture, the Ranger chassis can keep reporting `MOTION_MODE_PARALLEL` while all command topics are already zero. `robot_safety` observes `/ranger_base/status` for its outer mode-exit guard. The authoritative transition is now inside `ranger_base`: any probe or requested drive is held at zero until the firmware confirms DUAL_ACKERMAN, so a safety-layer probe cannot leak physical motion during mode change. Docking commands are exempt from the outer guard so fine docking can intentionally request lateral motion; the chassis core still performs the same confirmed transition.

The mode-exit guard is only an idle/stale-stream recovery path. While a normal
Nav2 command has been received inside `watchdog_timeout_sec`, the safety timer
must not inject its own zero merely because the confirmed chassis mode is
`MOTION_MODE_PARALLEL`; doing so would alternate fresh terminal side-slip with
zero at the timer rate. When the normal command stream stops or becomes stale,
the existing watchdog and mode-exit path still publish zero and return the
chassis toward DUAL_ACKERMAN. This reuses the existing command timestamp and
watchdog window and introduces no navigation or safety parameter.

Terminal pose recovery may need a small side-slip command after Ackermann MPPI reaches a lateral-dominant residual. API fallback remains bounded by `allow_api_lateral_cmd` and `api_lateral_max_mps`. The controller-native path is fail-closed: normal Nav2 `linear.y` remains zero unless `GoalScopedRotationShimController` publishes a fresh `/ranger_mini3/nav_terminal_lateral_enable` lease, and the accepted command is clamped by `normal_navigation_lateral_max_mps`. `ranger_base` derives and confirms the required lateral chassis mode from the final Twist.

For push-in spring charging docks, controlled undocking must be a continuous low-speed motion through the charger switch travel. `robot_safety` stores the last fresh `/cmd_vel_docking` command and republishes it from the safety timer while the docking-priority window is active, so watchdog/status refreshes do not insert zero commands between valid undock updates.

When `block_normal_motion_when_docked=true`, BMS contact, `/docking/status` docked/charging, or the persistent dock-contact latch blocks normal `/cmd_vel_collision_checked` output and publishes zero with `/safety/status=DOCKED_CONTACT_BLOCK`. A latch is treated as stale safety memory when fresh BMS says no contact and there is no current docked/charging status, so an old latch cannot permanently block navigation after a clean no-contact state is visible. `allow_docking_cmd_when_docked=true` keeps the docking channel available, but it no longer means that every docking Twist is legal after electrical contact. With `bms_docking_interlock_enabled=true`, the first fresh BMS contact immediately clears the cached docking command and publishes zero. That electrical-contact event is latched inside the final arbiter, so forward, lateral, and angular docking commands remain hard-blocked even if BMS messages later become stale or another publisher continues sending them. Only an exact zero or a pure negative-X command with a fresh controlled-undock reverse permit is accepted. The latch is released only after the current undock reports `undocked phase=succeeded`, the explicit reverse session ends, and fresh BMS feedback confirms no contact. Cancellation, motion-start failure, or no-progress failure cannot release it merely by disabling reverse. Status/permit arrival order is supported in either direction; an old success is not carried into a new attempt.

`/safety/dock_interlock_state` publishes the final arbiter's live BMS-contact,
private memory-latch, persistent-latch, and reverse-session state using reliable
transient-local QoS. `/safety/reconcile_dock_interlock` may clear only the private
memory latch, and only after the caller supplies outside-dock proof and the node
independently verifies fresh stable BMS no-contact, no docked status, no reverse
permit, and no fresh docking command. The API owns dock-pose geometry and the
persistent latch; `robot_safety` does not infer position or clear the persistent
file. See `docs/pre_navigation_dock_interlock_recovery.md` at repository root.

The reverse-permit release and fresh no-contact observations may arrive in
either callback order. Both callbacks evaluate the same release predicate, so
an already-disabled reverse permit converges when the later BMS no-contact
sample arrives without weakening any release condition.

When any blocking state is active, the node publishes a zero twist and latches the current safety state on `/safety/status`.

## P6 Mission And Elevator Motion Interlock

The P6 interlock cross-checks the operating-mode control plane without allowing
that plane to publish velocity. A `DOORWAY` or `ELEVATOR_RIDE` mode does not
authorize motion by itself; only the final arbiter can pass a command to
`/cmd_vel`.

### Owner-scoped motion hold

`SetMotionHold` identifies a hold by the exact `(owner, transaction_id)` tuple.
Acquire additionally requires a non-empty reason.

- Every current client sends a strictly increasing `command_sequence`.
  `robot_safety` remembers the highest accepted sequence for the exact key and
  rejects stale or legacy commands after sequencing has been observed. This
  prevents a timed-out late `RELEASE(N)` from clearing a newer cleanup
  `ACQUIRE(N+1)`; the response echoes the exact `applied_sequence` as proof.
- Holds are keyed by the exact owner and transaction. Different owners may
  intentionally share one transaction ID during a handoff, and all such holds
  compose. Motion remains blocked until every exact key has been released.
- An anonymous client or another owner cannot release a hold.
- A higher-sequence exact release against an already absent key installs a
  tombstone. An older acquire that arrives after a timeout is therefore stale
  and cannot recreate the hold.

Explicit elevator recovery must use
`ReleaseMotionHoldIfExecutionIdle`, not the ordinary release operation. The
arbiter compares the caller's final observed generation, proves both
`execution_session_engaged=false` and `execution_lease_active=false`, verifies
the exact hold and a strictly newer command sequence, then removes the hold in
one arbiter critical section. Any generation drift or newly acquired execution
resource leaves the hold in place.
- Repeating the same acquire is idempotent; changing its reason updates the
  existing record.
- A hold has no TTL. It remains active for the lifetime of the safety process
  until exact release. Restart is not a release protocol: startup still
  publishes zero, and recovery must reconstruct and validate mission state
  before any new motion.
- A hold dominates an otherwise healthy execution lease.

The current elevator-test path is mode-owned rather than execution-lease-owned.
Once an exact `robot_elevator_manager/elevator_<transaction>` special mode is
observed, safety retains that contract until an explicit mode release. Mode TTL
expiry therefore blocks motion; it does not silently fall back to ordinary
navigation. The execution-lease arbiter/service remains compatible with other
clients and historical recovery evidence.

The elevator failure order is always: acquire or confirm the hold, request
cancel of the one active Nav2 goal, wait for the action terminal state, then
verify fresh wheel/local odometry is settled. Destroying an FSM or returning a
failed action result must not release the hold.

### Execution lease and failure lock

Normal navigation behavior is unchanged while no P6 execution session is
engaged. Once `SetExecutionLease(OP_SET)` opens a session, the exact
`owner + mission_id + transaction_id + lease_id` must renew it before the
steady-clock TTL expires.

- Configured lease duration is bounded by `min_execution_lease_sec` and
  `max_execution_lease_sec`.
- Renewal is accepted only for the exact active tuple.
- An unrelated lease cannot take over an active session.
- Expiry retires the lease but deliberately leaves the session engaged.
  Consequently the arbiter remains in `EXECUTION_LEASE_MISSING` and continues
  publishing zero. It does not silently return to ordinary motion.
- Only the configured `execution_recovery_owner` may acquire a recovery lease
  for an expired, failure-locked session.
- Exact release closes a healthy or recovered session. Retired lease IDs cannot
  be reused.
- Releasing an exact, not-yet-observed lease ID also retires it. This
  release-before-delayed-set ordering prevents a timed-out old SET from opening
  a session after cleanup has completed.
- A healthy session permits the normal Nav2/collision-monitor source. An exact
  post-call transaction permit may additionally select the raw `/cmd_vel_nav`
  source described above; `/cmd_vel_api` and `/cmd_vel_docking` remain blocked
  for the whole session.
- `ELEVATOR_WAIT`, `ELEVATOR_RIDE`, `DOORWAY`, and `RECOVERY` require a fresh
  `/robot_mode/state` lease with the same owner and mission as the execution
  session. Safety uses the earlier of heartbeat timeout and advertised mode
  lease expiry; mode fallback to `NORMAL` or a lost heartbeat stops motion.

Execution TTL and reverse/lateral permit freshness are evaluated with
`std::chrono::steady_clock`, so ROS time pause or rollback cannot extend
physical motion authority. Any accepted interlock transition clears cached
API/docking commands, reverse/lateral permits, and starts a stop-dominant zero
window; an old command or permit cannot be replayed simply because a hold was
released.

Safety-stop publication is callback-synchronous, not deferred to
`publish_rate_hz`. A motion-hold or execution-lease service decision publishes
zero immediately whenever the interlock changed, and also whenever the final
combined snapshot is blocked even if the pure decision itself was unchanged.
An operating-mode state change follows the same rule: it clears cached
commands/permits and publishes zero in the mode subscription callback; an
unchanged but blocked mode snapshot also publishes zero. The timer remains the
watchdog and refresh path, not the first stop path.

`MotionInterlockState.motion_blocked` remains the low-level hold/execution-TTL
arbiter state. `interlock_effective_motion_blocked` additionally includes the
mode contract for the normal Nav2 source, while `normal_source_only` exposes
source restriction. `/safety/motion_allowed` remains authoritative for the
combined estop, localization, dock, watchdog, and interlock decision.

The owner strings coordinate components; they are not authentication. Without
SROS 2, deployment access control must not treat them as security identities.

### Isolated validation

The pure arbiter regression is
`test/test_motion_interlock_arbiter.cpp`. It covers composed holds, exact-owner
release, hold precedence, TTL expiry, failure lock, exact renewal/release,
conflicting leases, recovery ownership, and no-mutation rejection paths.

The base interlock ROS-graph smoke is:

```bash
bash /workspaces/njrh-v3/workspace1/src/robot_safety/test/run_isolated_interlock_smoke.sh
```

The normal Nav2 lateral/watchdog regression is:

```bash
bash /workspaces/njrh-v3/workspace1/src/robot_safety/test/run_isolated_normal_lateral_watchdog_smoke.sh
```

Do not invoke `isolated_interlock_smoke.py` directly. The wrapper fixes
`ROS_DOMAIN_ID=181`, sets `allow_reverse=true`, and starts a dedicated safety
node with normal, API, docking, output, mirror, operating-mode, and all reverse
or lateral permit topics under `/p6_test/*`. It deliberately lowers
`publish_rate_hz` to `1.0`: after synchronizing with the slow timer, the smoke
acquires an execution lease and requires finite zero output from the service
callback before the next one-second timer tick. It must still run with no live
runtime container, chassis bridge, or `/cmd_vel` connection. Its `EXIT` trap
terminates and waits for the isolated node and removes the temporary log on
success, failure, or interruption.

Do not invoke `isolated_normal_lateral_watchdog_smoke.py` directly either. Its
wrapper fixes `ROS_DOMAIN_ID=182`, remaps every topic under
`/lateral_guard_test/*`, and never connects the output to the real
`/cmd_vel`. It proves two paired requirements: a fresh permitted normal Nav2
side-slip stream contains no safety-timer zeros, and the same stream is forced
to zero after upstream refresh stops beyond `watchdog_timeout_sec`. The
wrapper's `EXIT` trap terminates the isolated node and removes its temporary
log.

The smoke test proves limited synthetic facts for the retained compatibility
arbiter: hold/release behavior, execution-lease expiry, API-source blocking during a healthy session,
normal-Nav2-source passage, owner/mission mismatch blocking, and advertised
mode-lease expiry taking effect before the heartbeat timeout. With global
reverse deliberately enabled, it also proves docking reverse cannot bypass the
docking-specific permit. Non-finite Twist commands injected independently on
the normal, API, and docking sources must all produce finite zero output. This
is not a hardware stop-distance test and is not currently registered as an
automatic launch test.

`robot_elevator_manager/test/test_nonmoving_cross_floor_scenario.cpp` also
composes this pure interlock with the mission, elevator, floor-transition,
mode, and correction-pause cores. It verifies the successful synthetic path
never acquires an execution session and finishes with no hold or mode lease. That GTest uses no ROS node, Twist
publisher, chassis connection, real Nav2 goal, or hardware motion.

### Real hardware release gate

Do not enable real elevator motion merely because the interfaces, unit tests,
or isolated smoke test pass. Hardware release additionally requires:

1. one and only one production `robot_safety` instance and `/cmd_vel` publisher;
2. verified hold-to-final-zero propagation through the ordinary
   `velocity_smoother -> collision_monitor -> robot_safety -> ranger_base`
   path and the transaction-scoped elevator-entry
   `velocity_smoother -> robot_safety -> ranger_base` path;
3. measured stop latency and stopping distance at the elevator speed limits;
4. process-kill and heartbeat-loss tests while a Nav2 goal is active;
5. proof that hold release does not replay cached nonzero API, docking, or Nav2
   commands;
6. restart recovery that cancels orphan goals and reacquires hold before any
   motion;
7. door-closing, stale observation, arm-not-stowed, localization-transition,
   and footprint-straddling fault injection;
8. an empty, stationary elevator acceptance before loaded or public operation.

Until the floor/elevator/mission ROS adapters, real floor assets, and the
arm/vision evidence sources are integrated, these interlocks are infrastructure
only and do not authorize a real elevator entry or exit.

## Parameters

- `watchdog_timeout_sec`: stop the robot when upstream control stalls
- `min_execution_lease_sec`: minimum accepted P6 execution TTL, default `0.20`
- `max_execution_lease_sec`: maximum accepted P6 execution TTL, default `5.0`
- `execution_recovery_owner`: only owner allowed to recover an expired execution session, default `robot_mission_manager`
- `motion_hold_service`: owner-scoped hold endpoint, default `/safety/set_motion_hold`
- `recovery_hold_release_service`: generation-fenced recovery endpoint, default
  `/safety/release_motion_hold_if_execution_idle`
- `execution_lease_service`: execution heartbeat endpoint, default `/safety/set_execution_lease`
- `motion_interlock_state_topic`: transient-local interlock diagnostics, default `/safety/motion_interlock_state`
- `execution_mode_state_topic`: operating-mode heartbeat cross-checked during special sessions, default `/robot_mode/state`
- `execution_mode_state_timeout_sec`: maximum mode-heartbeat age; advertised lease expiry may stop earlier, default `0.75`
- `cmd_vel_qos_depth`: Twist command stream QoS depth, default `1` for latest-only velocity control
- `zero_cmd_priority_enabled`: make near-zero Twist commands immediately stop-dominant for the active command owner
- `zero_cmd_priority_epsilon`: absolute per-axis Twist threshold treated as a zero command
- `zero_cmd_priority_burst_sec`: short window where final zero is repeated and late nonzero samples are rejected
- `docking_cmd_vel_in_topic`: docking/undocking command input
- `docking_cmd_priority_timeout_sec`: freshness window where docking input overrides normal input
- `publish_rate_hz`: safety refresh rate for watchdog zeroing and state publication
- `require_localization_health`: block motion until localization is explicitly healthy
- `publish_zero_on_startup`: force an initial zero command before any navigation source is active
- `block_normal_motion_when_docked`: zero normal commands while docked or charging
- `enable_bms_contact_guard`: use BMS charging-contact evidence for normal-motion blocking
- `enable_docking_status_guard`: use `/docking/status` prefixes for normal-motion blocking
- `enable_docked_latch_file_guard`: use the persistent docked latch file for normal-motion blocking
- `docked_status_prefixes`: lower-case status prefixes that mean docked/contact, default `docked,charging`
- `battery_state_topic`: BMS input for charging-contact evidence
- `docking_status_topic`: docking status input for docked/charging evidence
- `docking_contact_latch_file`: persistent explicit dock-contact state shared with API/docking manager
- `allow_docking_cmd_when_docked`: keep controlled `/cmd_vel_docking` motion available while normal motion is blocked
- `bms_docking_interlock_enabled`: on fresh BMS contact, immediately zero and latch rejection of subsequent forward/lateral/angular docking commands until confirmed undock
- `bms_docking_interlock_allow_reverse_undock`: while the BMS interlock is latched, allow only pure negative-X docking motion with a fresh docking reverse permit
- `bms_docking_interlock_reconcile_no_contact_sec`: continuous fresh BMS no-contact duration required before a proven remote-undock reconciliation, default `3.0`
- `dock_safety_interlock_state_topic`: reliable transient-local BMS docking-interlock state output, default `/safety/dock_interlock_state`
- `dock_safety_interlock_reconcile_service`: constrained no-motion reconciliation service, default `/safety/reconcile_dock_interlock`
- `spin_to_drive_settle_enabled`: hold linear drive briefly after confirmed actual SPINNING until the existing physical tail checks settle
- `spin_to_drive_odom_topic`: odom topic used for the actual yaw-rate settle check, default `/wheel/odom`
- `spin_to_drive_wz_threshold_radps`: actual yaw-rate threshold treated as stopped, default `0.02`
- `spin_to_drive_stable_samples`: consecutive settled odom samples required before releasing linear drive
- `spin_to_drive_require_local_odom_stable`: optional diagnostic gate for local EKF odom stability, default `false`; production release is based on raw wheel yaw-rate stability
- `spin_to_drive_local_odom_topic`: local odom topic sampled for diagnostics or for the optional local-stability gate, default `/local_state/odometry`
- `spin_to_drive_local_wz_threshold_radps`: local odom yaw-rate threshold treated as settled, default `0.03`
- `spin_to_drive_local_stable_samples`: consecutive local odom samples required only when `spin_to_drive_require_local_odom_stable=true`
- `spin_to_drive_local_stable_duration_sec`: minimum local odom stable duration only when the optional local gate is enabled, default `0.30`
- `spin_to_drive_local_yaw_delta_threshold_rad`: max local yaw drift across the stable window
- `spin_to_drive_local_odom_max_age_sec`: maximum accepted age of the local odom settle sample
- `spin_to_drive_require_imu_stable`: require the high-rate IMU yaw-rate tail detector before releasing linear drive after a spin, default `true`
- `spin_to_drive_imu_topic`: IMU topic used for the spin-tail detector, default `/lidar_imu_bias_corrected`
- `spin_to_drive_imu_wz_threshold_radps`: IMU yaw-rate threshold treated as physically stopped, default `0.035`
- `spin_to_drive_imu_stable_duration_sec`: minimum continuous IMU-stable duration before releasing linear drive, default `0.30`
- `spin_to_drive_imu_max_age_sec`: maximum accepted age of the IMU settle sample, default `0.10`
- `spin_to_drive_timeout_sec`: fail-open timeout after the first held linear-drive request, default `2.0`
- `spin_to_drive_linear_epsilon_mps`: existing translation-request threshold for handoff/mode-exit checks; never used to infer actual SPINNING
- `spin_to_drive_odom_max_age_sec`: maximum accepted age of the odom yaw-rate sample
- `mode_exit_guard_enabled`: guard normal/API drive commands from starting at full speed while the chassis still reports a lateral motion mode
- `mode_controller_status_topic`: compatibility parameter naming the actual Ranger mode status source, default `/ranger_base/status`
- `mode_exit_guard_probe_speed_mps`: bounded dual-Ackermann probe speed used to switch out of lateral mode, default `0.06`
- `mode_exit_guard_timeout_sec`: maximum probe duration before holding zero instead of passing the original command, default `1.0`
- `mode_exit_guard_status_max_age_sec`: maximum accepted age of the mode-controller status sample, default `0.5`
- `allow_reverse`: legacy/global shortcut for reverse outside an active execution session and outside the `DOCKING` source, default `false`; it never replaces the docking-specific permit
- `reverse_enable_topic`: bounded normal/API reverse permit, default `/ranger_mini3/allow_reverse`
- `docking_reverse_enable_topic`: controlled-undock reverse permit, default `/ranger_mini3/docking_allow_reverse`
- `teleop_reverse_enable_topic`: mapping-teleop reverse permit, default `/ranger_mini3/teleop_allow_reverse`
- `nav_terminal_reverse_enable_topic`: controller-native terminal reverse permit, default `/ranger_mini3/nav_terminal_reverse_enable`
- `nav_terminal_lateral_enable_topic`: controller-native terminal side-slip permit, default `/ranger_mini3/nav_terminal_lateral_enable`
- `reverse_enable_timeout_sec`: steady-clock freshness window for each reverse or lateral permit, default `0.75`
- `normal_navigation_reverse_max_mps`: absolute clamp for permitted terminal reverse, default `0.08`
- `normal_navigation_lateral_max_mps`: absolute clamp for permitted terminal side-slip, default `0.05`
- `elevator_navigation_reverse_max_mps`: reverse clamp selected only while the exact elevator operating-mode contract is valid, default `0.40`
- `elevator_navigation_lateral_max_mps`: lateral clamp selected only while the exact elevator operating-mode contract is valid, default `0.40`
- `allow_api_lateral_cmd`: allow bounded `/cmd_vel_api.linear.y` for API-owned terminal pose recovery, default `false`
- `api_lateral_max_mps`: absolute clamp for API lateral speed before final publication, default `0.10`

## Notes

- This package does not own planners, controllers, or collision monitoring. It only arbitrates the final command.
- The docking command hold is not a bypass: the held command is still published only by `robot_safety`, only while the docking command is fresh. Ordinary Nav2 reverse is limited to low-speed MPPI terminal correction and does not own docking/undocking motion.
- Strong persistent dock evidence survives restart and a transient/fresh BMS no-contact sample. It blocks every normal Nav2 command in both confirmed and uncertain on-dock states, while a fresh docking reverse permit still admits only pure negative-X controlled-undock commands. `POWER_SUPPLY_STATUS_FULL` alone neither creates nor restores the latch; it only supports an already retained docking session. The latch is cleared by proven controlled undock or explicit field confirmation.
- `allow_reverse=true` cannot authorize `DOCKING` reverse. Docking always requires a fresh `/ranger_mini3/docking_allow_reverse` permit; while a legacy execution session or the exact elevator operating-mode contract is active, reverse is restricted to the normal Nav2 source with its fresh terminal-reverse permit.
- Jetson runtime executes the compiled C++ node directly and fails fast if the binary is missing; the Python fallback path has been removed.
- `/cmd_vel_safe` is a diagnostic mirror when the runtime publishes the final command on `/cmd_vel`; the effective chassis command remains owned by `robot_safety`.

Built archives and the final executable require regression checks as well as
source review. See [elevator bypass build-artifact regression](docs/elevator_bypass_build_artifacts.md)
for the stale-library failure, artifact checker, and staged deployment boundary.
