# Docking Contact Stop Contract

This contract governs the transition from the final charging-contact push to the parked state. It does not change camera alignment, Nav2, AMCL, odometry integration, or the Ranger motion model.

## Required sequence

1. The first accepted BMS charging indication is timestamped. `robot_safety` independently clears any cached forward docking command on the same event and latches the contact interlock even if later BMS samples become stale.
2. A zero command is published immediately on `/cmd_vel_docking`.
3. The docking controller enters `ContactStopping` and republishes zero at the configured control rate.
4. `/motion_state` must provide a new, fresh official chassis-mode feedback message after the BMS event.
5. `/wheel/odom` must provide new, fresh feedback after the BMS event. Planar speed is `hypot(twist.linear.x, twist.linear.y)` and yaw speed is `abs(twist.angular.z)`.
6. Both speeds must remain below their thresholds for the configured duration and number of distinct wheel-odom samples.
7. Only after confirmation may reverse permission be disabled and Park be requested. Forward/lateral/angular docking commands remain blocked until an explicit pure-reverse undock session finishes and fresh BMS feedback confirms no contact.

The timeout is diagnostic only. Missing or moving feedback never authorizes Park; the node keeps publishing zero until a valid stop is confirmed. A stop-service request or undock request cannot bypass `ContactStopping`.

`/docking/status` uses a structured text contract: the first whitespace-delimited token is the state/event code and every later token is diagnostic metadata. API terminal-state classification must inspect only that leading code. Therefore both `contact_stopping ... contact_stop_feedback_timeout=false` and `contact_stopping ... contact_stop_feedback_timeout=true` remain non-terminal while zero commands continue; only an explicit failure code such as `contact_verify_timeout`, `contact_verify_failed_*`, or `undock_failed_*` may fail the job. This prevents field names or values containing `timeout`, `failed`, `not_found`, or `rejected` from changing the state-machine outcome.

After a terminal docking success, `robot_api_server` releases the localization bridge correction pause on a deferred worker rather than waiting inside the `/docking/status` subscription callback. The API uses a single-threaded ROS executor, and a synchronous service wait inside that callback would starve its own response completion for `service_timeout_sec` and leave a false delayed-side-effect record even when the bridge applied the request immediately. This service-response bound is independent of the physical contact-confirmation timeout; increasing the latter does not repair executor starvation.

## Runtime parameters

| Parameter | Runtime value | Purpose |
|---|---:|---|
| `contact_stop.motion_state_topic` | `/motion_state` | Official motion-mode feedback and freshness witness |
| `contact_stop.wheel_odom_topic` | `/wheel/odom` | Actual chassis twist used for stop confirmation |
| `contact_stop.feedback_max_age_s` | `0.50` | Maximum receive age for either feedback stream |
| `contact_stop.linear_speed_threshold_mps` | `0.01` | Maximum planar speed considered stopped |
| `contact_stop.angular_speed_threshold_radps` | `0.02` | Maximum yaw rate considered stopped |
| `contact_stop.stable_duration_s` | `0.50` | Required continuous stopped duration |
| `contact_stop.stable_samples` | `5` | Required distinct stopped wheel-odom samples |
| `contact_stop.feedback_timeout_s` | `3.0` | Warning threshold; it does not force completion |

## API BMS configuration ownership

`robot_api_server/features/power/power_configuration_module` is the sole ROS
parameter declaration owner for the BMS topic, freshness, electrical-contact
thresholds, full-SOC interpretation, and
`dock_contact_latch_bms_require_contact_sec`. Docking receives the validated
current/SOC values as immutable inputs; it does not redeclare them. The
historical `teleop_charging_current_min_a` parameter name is retained for
deployed YAML compatibility, but it represents shared BMS contact evidence.
The separate `teleop_stop_on_charging` switch remains a teleop policy. This
ownership move does not alter any default, clamp, contact decision, latch clear
rule, or command path.

## Telemetry

`DOCK_BRAKE_START` records the BMS receive time, first-zero publish time, ContactVerify travel/time at BMS, and feedback sequence baselines. `DOCK_BRAKE_CONFIRMED` records the first below-threshold feedback time, confirmation time, wheel-odom distance after BMS, zero-command count, final measured speeds, and motion mode. The same values are retained in `/docking/status` with `brake_confirmed=true` after completion.

`first_zero_to_stop_ms` is the observed time from the first zero publish to the first sample in the final stable stopped window. `first_zero_to_stop_confirmed_ms` includes the configured stability dwell and is therefore intentionally larger.

The isolated regression test remaps every command and state topic under `/njrh_test` and therefore cannot reach the chassis command chain:

```bash
bash scripts/jetson/runtime_overlay/scripts/test_docking_contact_stop_contract.sh
bash scripts/jetson/runtime_overlay/scripts/test_robot_safety_bms_docking_interlock.sh
bash scripts/jetson/runtime_overlay/scripts/test_docking_contact_slow_zone_contract.sh
```
