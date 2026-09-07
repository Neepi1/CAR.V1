# FloorSwitch correction-pause handoff

## Problem fixed on 2026-08-05

The elevator runtime and `robot_floor_manager` intentionally overlap their
owner-scoped localization-correction pauses while an atomic floor switch starts.
The floor manager first proves its own pause and invalidates the source runtime
context. Only then may the elevator runtime release its caller-owned pause.

The former Action feedback contract represented that barrier as one transient
stage string:

```text
CALLER_PAUSE_HANDOFF_READY -> VERIFY_PAUSE_HANDOFF
```

`robot_api_server` stored only the most recent string. If both feedback samples
arrived in one executor burst, `VERIFY_PAUSE_HANDOFF` overwrote the ready edge
before the waiting thread observed it. The caller pause therefore remained held,
the floor manager timed out while waiting for its release, and the action
aborted before map loading began.

## Current contract

`FloorSwitch.action` feedback now carries:

- `transaction_id`: exact transaction identity;
- `stage_sequence`: monotonic floor-transition effect sequence;
- `caller_pause_handoff_ready`: a level-triggered readiness flag.

The readiness flag is true both while publishing the handoff barrier and while
the floor manager waits for caller release. `robot_api_server` feeds feedback
through `FloorSwitchHandoffTracker`, which:

- accepts only the active transaction and submission generation;
- rejects lower, out-of-order stage sequences;
- treats duplicate feedback idempotently;
- latches readiness monotonically until the submission is reset;
- distinguishes a terminal result received before readiness;
- also recognizes the exact transaction's reliable floor-status
  `CALLER_PAUSE_HANDOFF_READY` or `VERIFY_PAUSE_HANDOFF` stage if Action feedback
  delivery is missed.

After readiness, the existing elevator FSM releases only
`robot_elevator_manager:<transaction_id>`. The floor manager still requires
fresh `CorrectionPauseState` evidence proving that the caller key is absent,
the exact `robot_floor_manager:<transaction_id>` key remains, and correction
pause remains effective before it mutates any map asset.

## Invariants unchanged

- The floor switch remains atomic and fail-closed.
- No navigation, localization, tolerance, MPPI, costmap, or speed parameter is
  changed by this fix.
- `map -> odom` and `odom -> base_link` ownership are unchanged.
- The App still cannot switch maps or publish chassis velocity directly.
- A retained recovery journal is never deleted or edited to bypass recovery.

## Verification

Run the isolated unit tests for `floor_transition_executor` and
`floor_switch_handoff_tracker`, then build and test `robot_interfaces`,
`robot_floor_manager`, and `robot_api_server`. Deployment requires one complete
`njrh-runtime.service` restart; individual navigation-node restarts are not
allowed. The first physical confirmation remains a stationary, operator-driven
elevator test and must verify that caller-pause release occurs before the floor
action leaves `VERIFY_PAUSE_HANDOFF`.
