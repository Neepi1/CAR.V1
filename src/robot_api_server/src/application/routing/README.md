# Application HTTP routing

`ApplicationRouterModule` owns the authenticated robot API routing graph:

- capture admission epochs only for elevator endpoints;
- keep the elevator test admission/interlock out of unrelated HTTP handlers;
- preserve the exact system-status, maps, elevator, mapping, metadata,
  subscriptions, safety, floor-switch, localization, navigation, and docking
  precedence;
- preserve reserved mapping/navigation `501` responses and the final `404`.

It contains no HTTP socket, authentication, ROS, TF, navigation, docking, or
elevator algorithm. The composition root injects narrow handlers from the
gateway and feature modules. Individual feature modules continue to own their
endpoint matching and business semantics.

`application_router_wiring.cpp` is the single concrete adapter from the
composition root's feature instances to those narrow route ports. This keeps
the root constructor declarative without making transport infrastructure
depend on robot features.

Non-elevator handlers retain the legacy epoch argument (zero) for binary
compatibility, but never acquire the elevator fence. Mapping start/save, maps,
navigation, docking, floor switching, localization, safety HTTP and teleop no
longer use test admission. Domain data/asset mutexes and the bottom safety chain
are unchanged. This is not a removal of all robot or file-system mutexes.
