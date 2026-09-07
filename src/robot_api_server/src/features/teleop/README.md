# Teleop API

This module owns the complete API-facing mapping-only WebSocket teleop
vertical slice:

- the `/ws/v1/teleop` route, token/upgrade validation, handshake, frame loop,
  and disconnect cleanup;
- decoding the existing `cmd_vel` / `stop` payload shapes and legacy aliases;
- applying the configured linear/angular limits and producing the existing
  acknowledgement JSON;
- tracking concurrent sessions and the latest accepted velocity;
- owning `/cmd_vel_api` and teleop reverse-permit ROS publishers plus the
  watchdog repeat timer;
- enforcing the existing mapping, elevator-execution, charging, and motion
  admission rules; and
- acquiring/refreshing/releasing the existing `teleop` and `tf` subscription
  leases.

`TeleopModule` receives only cross-domain ports for token checking, elevator
interlock/admission, charging contact, mapping and pose snapshots, subscription
lease operations, and process lifetime. Generic HTTP response and WebSocket
frame transport remain in `infrastructure/http`; the composition root does not
retain teleop policy wrappers or ROS teleop resources.

`TeleopConfigurationModule` owns all 11 `teleop_*` ROS parameters and applies
the established velocity/timeout bounds. The subscription module projects its
already-normalized maximum lease TTL into Teleop, so Teleop does not redeclare
or reinterpret subscription-owned parameters.

Commands continue through `/cmd_vel_api` and `robot_safety`; the App never
publishes chassis velocity directly. A final session disconnect, explicit
`stop`, charging guard, or execution interlock still clears the cached command
and publishes zero. This ownership move adds no gate and changes no velocity,
timeout, DDS, TF, mapping, elevator, docking, or mechanical-arm parameter.
