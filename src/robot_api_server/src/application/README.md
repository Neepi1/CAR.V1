# Application layer

Coordinates feature modules without owning ROS transport details or domain
algorithms.

- `composition/` owns the one application object graph: configuration-module
  invocation, top-level aggregate construction, narrow dependency wiring,
  late-dependency completion, gateway start, and dependency-safe shutdown.
- `runtime_configuration/` owns the three shared construction-time ROS
  parameters and projects one immutable value set into neighboring features.
- `routing/` owns authenticated endpoint precedence, the request-scoped motion
  admission epoch, global elevator interlock ordering, and final 501/404 policy.
- `runtime_mode/` owns the atomic mapping/navigation/docking state and the
  single-owner transition token.
- `subscriptions/` owns the App page-resource HTTP lifecycle, TTL/refcounts,
  and page-scoped scan cache.

The six-line process entry delegates to the process bootstrap, which constructs
`ApplicationCompositionModule`. The composition module asks the dedicated
configuration modules to declare parameters; it does not duplicate those
declarations, feature policy state, or the route graph.
