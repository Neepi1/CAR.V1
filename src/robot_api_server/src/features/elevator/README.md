# Elevator

- `elevator_module` is the API-facing module boundary. It owns all
  `/api/v1/elevator-config*` and `/api/v1/elevator-test*` routes, constructs
  the configuration and execution components, owns the recovery interlock and
  motion-admission fence, and binds exact map assets through injected catalog
  and runtime-observation ports.
- `configuration/` owns commissioning drafts, validation, immutable releases,
  rollback, exact map references, and the separate construction-time runtime
  parameter graph. The latter declares parameters only and never calls the
  mechanical-arm service.
- `execution/` owns the App test transaction adapter, elevator runtime policy,
  recovery sequencing, and the loopback arm-service client boundary.

The mechanical-arm black box at port 8083 is an external service and is never
implemented or modified here.

The API composition root supplies only neighboring facts (runtime idleness,
fresh map pose, floor-transition interlock, and delayed-effect count). It does
not own elevator transactions, routes, ROS execution objects, or arm calls.
