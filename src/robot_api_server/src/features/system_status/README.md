# System status

Owns the complete read-only API status slice:

- `GET /api/v1/status`, including the stable aggregate JSON contract for
  runtime mode, mapping, navigation, docking, safety, localization, BMS,
  subscriptions, and HTTP capacity;
- `GET /api/v1/robot/pose`, including runtime-map readiness failures, fresh
  `map -> base_link` pose output, and exact map identity;
- the single process-resident `/floor_manager/status` subscription and its
  thread-safe cache.

`system_status_wiring` owns the complete read-only projection from neighboring
domain modules into those ports: mapping/runtime/safety/BMS/docking/localization,
navigation, floor-switch, map identity, subscriptions, and HTTP capacity. The
composition root supplies only concrete module references and the shared bridge
goal-readiness evaluator.

This module does not subscribe to TF or BMS, select a map, alter localization
readiness, mutate runtime state, publish safety state, or command motion. The
extraction preserves the existing response fields and readiness precedence and
adds no gate.
