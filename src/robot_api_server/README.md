# robot_api_server

`robot_api_server` is the production-facing HTTP gateway for Android and other non-ROS clients.

It does not own mapping, localization, navigation, or chassis control logic. It only exposes a narrow HTTP API and forwards requests into existing ROS 2 topics and services.

## Code layout

The implementation and public headers mirror the same domain-oriented tree:

```text
features/
├── system_status/
├── maps/
│   ├── catalog_activation/
│   ├── poses/
│   └── keepout/
├── mapping/
│   └── runtime/
├── localization/
├── floor_switch/
├── navigation/
│   ├── configuration/
│   ├── runtime/
│   ├── mission/
│   └── terminal_control/
├── elevator/
│   ├── configuration/
│   └── execution/
├── power/
├── docking/
│   ├── configuration/
│   ├── lifecycle/
│   └── predock_alignment/
├── safety/
└── teleop/

application/
├── composition/
├── routing/
├── runtime_configuration/
├── runtime_mode/
└── subscriptions/

infrastructure/
├── http/
└── process/
```

Every directory has a local `README.md` that states what it owns and what
still remains in the composition root. The same relative layout is used under
`include/robot_api_server/`, `src/`, and (where tests exist) `test/`.

- `src/robot_api_server_node.cpp` is a six-line process entry. Its only action
  is forwarding `argc/argv` to
  `infrastructure/process/robot_api_process`; it owns no ROS object, parameter,
  module, route, state, worker, or business policy.
- `infrastructure/process/robot_api_process` owns ROS initialization, the
  concrete `RobotApiServerNode`, the single-threaded executor, the established
  transient action-client exception retry, and process shutdown.
- `application/composition/application_composition_module` is the only object
  graph and lifecycle root. It invokes the dedicated configuration modules,
  constructs each top-level feature aggregate, connects narrow cross-domain
  ports, completes the deliberate late dependencies, starts the gateway, and
  shuts modules down in dependency-safe order. It contains no feature handler
  or domain algorithm.
- `application/routing/application_router_module` owns the complete
  authenticated business routing graph, including one request-scoped motion
  admission epoch, the global elevator interlock, exact module precedence, and
  reserved/unknown fallback responses. Its wiring adapter binds concrete
  feature instances without putting robot dependencies into HTTP transport.
- `infrastructure/http/api_gateway_module` owns the complete public gateway
  lifecycle and policy: parameter/environment token resolution, preflight and
  authentication, transport callback wiring, access/event logging,
  WebSocket-session shutdown ordering, connection observations, and the exact
  `/api/v1/openapi` document. It delegates one already-authenticated request to
  the application router.
  `api_gateway_configuration_module` uniquely declares the four public gateway
  ROS parameters; environment-token fallback and connection normalization
  remain runtime policy inside `ApiGatewayModule`.
- `include/robot_api_server/infrastructure/http/http_server.hpp` plus
  `src/infrastructure/http/http_server.cpp` form the HTTP transport module. It
  receives normal routing, socket-route, access-log, and event-log callbacks;
  it has no ROS or robot-domain dependency. See
  `src/infrastructure/http/README.md` for the ownership contract.
- `features/maps/maps_module` owns the complete API-facing maps vertical slice:
  catalog startup recovery/migration, activation journals and integrity state,
  all map/semantic/pose/keepout routes, and keepout ROS runtime proof.
  `maps_configuration_module` uniquely declares the 15 maps, shared-path, and
  keepout parameters and emits the full module config plus one immutable path
  projection; `catalog_activation/map_runtime_state_store` owns runtime-context
  and last-navigation-selection file IO. `maps_feature_module` owns both that
  store and the complete maps module, including the cross-asset commit mutex
  and neighboring runtime ports. The root retains neither individual maps
  objects, parallel maps path scalars, nor map-state persistence methods.
  `features/maps/catalog_activation/` owns its immutable identity and filesystem
  mechanisms; `features/maps/poses/` owns `poses.yaml`; `features/maps/keepout/`
  owns semantic/raster transactions. The root only composes cross-domain ports.
  `features/mapping/` is one deep
  API-facing module: it owns the mapping HTTP routes, asynchronous start
  transaction, mapping process lifetime, live `/map` subscription/cache,
  exact `/scan` owner restoration, status projection, and saved-map asset
  generation. `mapping_configuration_module` is the sole owner of its 10 ROS
  parameter declarations and emits the full mapping config from explicit
  maps-owned path inputs. `mapping_feature_module` owns the module's complete
  navigation/floor/elevator handoff graph. It does not own either mapping algorithm.
- `features/localization/localization_feature_module` is the aggregate owner
  of the full localization family. It creates `localization_module` first,
  then attaches `post_relocalization_settle_module` after navigation exists;
  both phases complete before HTTP starts. The root owns only this aggregate
  and no localization submodule or localization port graph.
  `features/localization/localization_module` owns the API-facing
  localization slice: resident TF observation and pose composition,
  localization-result and bridge-status caches, Isaac trigger/acceptance,
  bridge-correction pause, AMCL no-motion/refinement, and the manual
  localization route. `localization_configuration_module` is the single owner
  of all 48 API-side localization and settle parameter declarations; it
  preserves frame normalization and dependent bounds and emits the existing
  module/settle configs plus the floor-health projection. Its sibling
  `post_relocalization_settle_module` owns the
  accepted-sequence stability transaction, state/evidence cache, TF/bridge/
  local-costmap checks, timeout/cancel result, and the established post-undock
  warning-only exceptions. The aggregate supplies only narrow cross-domain
  ports. The root retains no parallel localization frame or
  freshness scalars. All three modules only consume TF; `map->odom` remains exclusively owned by
  `robot_localization_bridge` and `odom->base_link` by `robot_local_state`.
- `features/floor_switch/` owns the complete floor-switch API vertical slice:
  all four HTTP routes, live Action and offline service clients, transaction
  worker/cancellation, exact-map selection proof, retained floor/localization
  health subscriptions and interlock, handoff tracking, and runtime-map-context
  persistence. `floor_switch_feature_module` owns the module and its complete
  runtime/admission port graph. Its configuration module is the sole owner of all 6 floor
  status/switch parameter declarations and emits the complete module config
  plus the status-topic projection. The composition root supplies only cross-domain runtime/map/
  elevator ports; `robot_floor_manager` remains the atomic switch authority.
- `features/navigation/navigation_module` owns the complete API-facing
  navigation slice and dispatches every `/api/v1/navigation/*` route. Its
  `navigation_state` projection preserves bridge-over-AMCL readiness,
  transition-versus-recovery semantics, dock/safety/settle evidence, and the
  canonical goal/cancel job JSON. `configuration/` is the single declaration
  and composition boundary for all 112 navigation ROS parameters and emits the
  existing module, terminal-runtime, goal-execution, and goal-executor config
  graphs; `runtime/` owns process/action/bridge/cancel runtime state;
  `mission/` owns goal admission, goal jobs, and commercial completion
  decisions; `terminal_control/` owns bounded post-Nav2 correction.
  The composition root supplies only cross-domain evidence and effects; it no
  longer owns a navigation route, state handler, or parallel navigation
  parameter scalars.
- `features/navigation/navigation_feature_module` is the one top-level owner
  of the complete ordinary-navigation runtime family. It constructs the
  navigation HTTP/runtime module, terminal ROS adapter, goal transaction, and
  goal executor with one validated dependency graph. The application
  composition module owns only this aggregate and exposes its narrow accessors
  to docking and status composition; neither the process entry nor process
  bootstrap owns navigation Ports or parallel submodule pointers.
- `features/navigation/terminal_control/navigation_terminal_runtime_module`
  owns the terminal-control ROS edge: final command, Nav2 speed-limit and
  reverse-permit publishers; Ranger mode, wheel odometry, local-costmap and
  `/rosout` subscriptions; stop/mode acknowledgements; and synchronized
  costmap/drop evidence. It implements `NavigationTerminalRuntimePort` for the
  deterministic terminal controller. `NavigationFeatureModule` connects its
  existing goal/docking executor ports; neither the process entry nor the
  application composition module owns a duplicate terminal publisher,
  subscription, or cache. Existing topics, QoS,
  timeouts, thresholds and success decisions are unchanged, and no gate was
  added.
- `features/elevator/elevator_module` owns the complete API-facing elevator
  slice: configuration/test routes, component lifetime, execution interlock,
  motion-admission fence, ROS runtime adapter, and exact map binding.
  `configuration/` owns commissioning assets and immutable releases; its
  `ElevatorRuntimeConfigurationModule` also uniquely declares the 15 existing
  adapter/arm-client/scoped-BT/permit/recovery parameters and combines them
  with explicit neighboring-domain inputs into `ElevatorModuleConfig`;
  `execution/` owns the test transaction, arm black-box client boundary,
  runtime policy, and recovery sequencing. The composition root injects only
  neighboring runtime facts, and no code here modifies the arm service. The
  root retains no elevator-owned parameter declaration or partial config copy.
- `features/docking/docking_feature_module` is the single top-level owner of
  the complete docking runtime family. It owns all docking submodules, the
  canonical job store, the docking worker queue, and start serialization. A
  deliberate core/completion construction sequence resolves the existing
  Navigation/Docking dependency cycle before the gateway starts; it is not a
  runtime fallback. The application composition module retains one aggregate
  pointer and uses narrow accessors for Navigation, status, power evidence,
  routing, and ordered shutdown. No docking Ports or individual submodule
  pointers remain in the composition root or process bootstrap.
- `features/docking/lifecycle/` owns docking job/status models, all six
  App-facing docking HTTP transactions, and the complete return-to-dock
  orchestration (runtime readiness, pre-dock Nav2, BMS contact stop, and
  localization barriers).
  `DockingJobStore` is the single owner of the mutable job, mutex, sequence,
  exact state JSON, and terminal commit; HTTP, executor, and status modules
  share that store and cannot diverge into parallel task state.
  `DockingCorrectionPauseModule` owns applying and releasing the
  `docking_fine` bridge pause, its canonical job/display-pose projection,
  protection of a live fine-docking owner, and stale-pause cleanup. The
  docking aggregate supplies bridge, pose, and logging ports.
  `DockingStatusModule` separately owns `/docking/status` terminalization,
  deferred bridge-correction-pause release, and the post-fine/post-undock
  relocalization worker while sharing the one canonical docking job.
  `DockingRuntimeModule` owns the docking-manager child process, all three
  docking Trigger clients, status and selected observation subscriptions,
  observation evidence cache, `/cmd_vel_docking` and Ranger-mode publishers,
  and the single execution worker. It preserves the existing QoS, timeout,
  charging-retry, source-filter, and delayed-side-effect semantics and adds no
  gate. `DockContactInterlockModule` owns the complete persistent latch and
  pre-navigation dock classification: source strength, BMS session evidence,
  full-charge idle, confirmed-undock auto-clear, occupancy decision, and exact
  diagnostic JSON. The docking aggregate injects runtime/BMS/job observations
  and maps the immutable result into neighboring modules. Its sibling
  `PreNavigationUndockModule` owns the complete controlled-undock-before-Nav2
  transaction: single-start serialization, canonical job creation, manager and
  Trigger-service handoff, service/status evidence, bounded wait, and optional
  post-undock localization-readiness proof. `DockingJobExecutionModule` implements the
  complete `DockingJobExecutionPort`: canonical job-store access, serialized
  pre-dock Nav2 submission and unresolved-side-effect evidence, cancellation,
  runtime/localization/BMS/terminal-control effects, and command-owner conflict
  reporting. `DockingFeatureModule` owns that interface;
  `DockingConfigurationModule` is the single declaration/composition boundary
  for all 100 docking, predock, fine-entry, dock-latch and undock
  ROS parameters. It applies the unchanged defaults, clamps and dependent
  bounds and emits the concrete configs consumed by the ten docking units;
  the root retains no parallel docking scalar configuration;
  `predock_alignment/` owns pure approach-frame geometry and the complete
  stateful staging transaction: handoff evidence, bridge settle/correction
  freeze, recoverable-residual admission, fine-entry validation, and the
  `/docking/start` handoff. `DockingJobExecutor` calls only the module's
  semantic interface. ROS pose acquisition, safety, shared command ownership,
  drive mode, motion, bridge, observation, floor, and service effects are
  injected ports. The production default delegates residual yaw/lateral motion,
  so `robot_docking_manager` is the single near-field physical owner.
- `features/power/` owns the complete process-resident `/battery_state` edge:
  its ROS subscription, normalized BMS snapshot, contact/no-contact timing,
  freshness expiry, and ordered evidence notifications. Its configuration
  module uniquely declares all 8 BMS evidence parameters, including the
  contact-stability duration formerly declared beside the docking latch, and
  emits the existing `PowerModuleConfig`. The legacy
  `teleop_charging_current_min_a` name remains compatible while
  `teleop_stop_on_charging` stays a teleop policy. The composition root has no
  parallel BMS parameter scalar or raw-state cache. This extraction changes no
  contact rule, timeout, admission gate, or motion owner.
- `features/system_status/` owns both read-only system routes
  (`/api/v1/status` and `/api/v1/robot/pose`), the full aggregate JSON
  projection, and the one resident `/floor_manager/status` subscription. The
  accompanying wiring adapter owns immutable domain-snapshot and pose/map
  lookup projection; the composition root passes concrete module references
  only. No parallel floor-status cache, snapshot assembly, or status handler
  remains there.
  This extraction adds no readiness or motion gate.
- `application/subscriptions/` owns the complete App-scoped subscription
  slice: all three HTTP routes, compatibility parsing, TTL/refcount expiry,
  resource transitions, and the page-owned high-rate `/scan` cache. The root
  supplies only resident-status, live-map, resident-TF, and teleop-clear ports.
  Its configuration module uniquely declares the four scan/TTL parameters;
  runtime normalization and Teleop's normalized TTL projection are unchanged.
  `application/runtime_mode/` owns the atomic mapping/navigation/
  docking business snapshot, its mode priority, docking state commits, and the
  single-owner asynchronous mode-transition token.
  `application/runtime_configuration/` uniquely declares the shared Nav2
  action name/status topic and base ROS service timeout, then projects the same
  immutable values into all consuming feature configuration graphs.
- `infrastructure/http/` owns the gateway lifecycle/policy, generic transport,
  and HTTP/JSON primitives;
  `infrastructure/process/` owns deferred execution and child-process helpers.
- `features/teleop/` owns the complete WebSocket teleop vertical slice:
  socket route/session lifecycle, payload decoding/clamping, mapping/elevator/
  charging admission, subscription leases, concurrent command state,
  `/cmd_vel_api` and reverse-permit publishers, and watchdog repetition.
  `infrastructure/http/` owns the shared token decision and generic
  HTTP/WebSocket transport; the composition root supplies cross-domain
  observation and admission ports.
- `features/safety/` owns the complete App-facing safety slice: all three ROS
  topic parameters, process-resident safety subscriptions, `/safety/estop`
  publisher, both stop/resume HTTP routes, immutable status/JSON model, the
  floor/elevator-atomic resume transaction, and the ordered emergency stop
  fan-out for an unproven Nav2 terminal result. The composition root only
  injects cross-domain admission and stop-effect ports. Existing
  `COMMAND_STALE` versus hard-block interpretation is unchanged, and final
  command arbitration remains in `robot_safety`.

## Boundaries

- Safety stop and resume publish `std_msgs/Bool` to `/safety/estop`.
- Robot battery state is read from Ranger's `/battery_state` (`sensor_msgs/BatteryState`) and exposed in `/api/v1/status` as `bms.soc`, power-supply fields, `bms.present`, `bms.charging_contact`, and `bms.charging_contact_reason`.
- A different-map offline selection calls `/floor_manager/switch_floor` with
  the exact immutable
  `building_id/floor_id/map_id/asset_epoch/asset_digest` when
  `resume_navigation=false`. The gateway verifies the echoed identity and
  dynamic source paths before its serialized `current/` activation. It holds
  the cross-process asset commit lock across source verification, the service
  proof, a second epoch/digest verification, and activation; post-preflight
  drift is rejected as `FLOOR_SELECTION_SOURCE_DRIFT`. An exact confirmed
  same-map compatibility no-op returns before the floor service.
- Ordinary same-floor navigation startup is exposed only as
  `POST /api/v1/navigation/start` after exact offline map selection. It
  verifies the immutable map identity and fixed `current/` projection before
  starting or reusing the repository-owned runtime. A fresh
  `runtime_map_context.state=starting` is reusable only while the API-owned
  process, the Nav2 action server, or the critical lifecycle set still proves
  a live runtime owner. A stale ownerless context never produces a reused
  `202`; the gateway starts a new owned runtime instead.
- The legacy `/api/v1/floors/switch` path remains offline-only and rejects
  `resume_navigation=true`. Manual live switching uses
  `POST /api/v1/floor-switch/start`, polls
  `GET /api/v1/floor-switch/state`, and optionally requests
  `POST /api/v1/floor-switch/cancel`. These endpoints submit only the strict
  `/floor_manager/floor_switch` Action and never stop/restart the resident
  navigation runtime or fall back to offline selection.
- Localization trigger calls `/global_localization/trigger`.
- Map listing reads released floor assets and runtime flat map files.
- `POST /api/v1/mapping/2d/start` starts the repository-owned `slam_toolbox` 2D mapping runtime chain.
- `POST /api/v1/mapping/2d/stop` terminates the App-started 2D mapping chain without stopping common services.
- `POST /api/v1/mapping/2d/save` saves the current `slam_toolbox` occupancy grid as flat runtime assets plus a structured floor bundle, then terminates the mapping chain without selecting that map for navigation.
- `POST /api/v1/mapping/stop` and `POST /api/v1/mapping/save` are REST aliases for the same 2D mapping stop/save operations.
- `POST /api/v1/maps/delete` deletes only inactive, non-runtime-bound saved maps that are not referenced by the current elevator configuration. Active/runtime-bound targets fail with `ACTIVE_MAP_DELETE_DISABLED`; elevator-bound targets fail with `ELEVATOR_CONFIG_MAP_IN_USE`. Before deletion it re-resolves the exact building/floor/map root, rejects every symlinked ancestor, atomically renames the map to a same-parent tombstone, fsyncs the namespace, removes the tombstone, and fsyncs again. The endpoint never auto-activates a remaining map.
- `GET /api/v1/robot/pose` returns only a fresh `map -> base_link` TF pose. It never falls back to `/odom` or wheel odom. The server keeps its `/tf` subscription resident at process startup; pose requests and App page leases must not create and destroy reliable `/tf` subscribers because that can churn Fast DDS endpoints and stall the `robot_localization_bridge` TF publisher.
- `GET /api/v1/maps/semantic_layer` reads the backend-owned editable map overlays for the selected `map_id`.
- `GET /api/v1/maps/poses` reads semantic delivery points from the selected `maps/<map_id>/poses.yaml`.

## Manual live floor switch

The robot-detail App workflow may change floors while the navigation and
localization runtime remains resident. It does not permit an active Nav2 goal:
the strict Action first proves Nav2 idle and stable wheel/local odometry stop,
then owns the motion hold, localizer reload, bridge fence, explicit target
localization, costmap refresh, and exact runtime-context commit.

`POST /api/v1/floor-switch/start` accepts only
`building_id/floor_id/map_id`. The server resolves and freezes the immutable
`asset_epoch/asset_digest`, creates one transaction ID, and submits that exact
identity to `/floor_manager/floor_switch`. `GET /api/v1/floor-switch/state`
requires that transaction ID. `POST /api/v1/floor-switch/cancel` requests
Action cancellation but remains nonterminal until the Action reports an
explicit terminal result.

Admission requires the source runtime context to be `ready`, confirmed, and
backed by a nonzero explicit-localization sequence. A request received during
resident-navigation cold start is rejected before any Action, motion-hold, or
correction-pause command is submitted. If the frozen target already equals
the exact confirmed runtime identity, the HTTP transaction completes
idempotently without reloading the same asset through its immutable and
`current/` projections.

The client may display success only when the returned target and active
`building_id/floor_id/map_id/asset_epoch/asset_digest` all equal the frozen
identity, `runtime_context_valid=true`, `recovery_required=false`, and the
failure code is zero. Admission/result timeout is `UNKNOWN`, never success;
the API retains that transaction as recovery-required and will not submit a
replacement floor switch. No endpoint in this workflow sends a navigation
goal or chassis velocity.

## Elevator configuration management

The commissioning API is deliberately separate from ordinary semantic points:

```text
GET  /api/v1/elevator-config?building_id=<building>[&release_id=<release>]
PUT  /api/v1/elevator-config/draft
POST /api/v1/elevator-config/publish
POST /api/v1/elevator-config/rollback
```

A current schema-v3 building draft contains one or more elevators. Every served
floor binds an exact `floor_id` and `map_id`, exactly four map-frame poses
(`hall_call`, `landing`, `cabin`, `cabin_panel`), and explicit
`hall_call_panel_side`/`cabin_panel_side` values (`LEFT` or `RIGHT`). Side is
defined relative to the robot heading at the corresponding commissioned pose.
Schema v3 enforces `landing.yaw ≈ hall_call.yaw + pi`, a straight reverse
`landing -> cabin` axis, and aligned `cabin`/`cabin_panel` yaw. The explicitly
commissioned `cabin_panel_side` is authoritative: publication does not infer
LEFT/RIGHT from map XY or require the two measured poses to form a pure lateral
vector. Runtime closes the actual residual against the stored panel pose.
Schema v2 remains readable/executable for already published three-point
releases. Neither version accepts operator-authored threshold, door-jamb, or
clearance geometry. Draft save is allowed while incomplete and returns
structured issues. Publish always revalidates:

- path-safe and unique IDs;
- at least two floors and all required roles/sides for the selected schema;
- finite pose coordinates and map bounds with rotated map origins;
- exact building/floor/map ownership and all required nav/localizer assets;
- the bound map content digest;
- `robot_elevator_manager` topology validation and generated YAML round-trip.

Draft and release mutations use optimistic `expected_draft_revision` and
`expected_release_id`; stale clients receive `409`. Published releases are
immutable under:

```text
maps_release/<building_id>/.elevator_config/
  draft.json
  drafts/<draft_revision>/
    configuration.yaml
    validation.json
  current -> releases/<release_id>
  current.json -> current/current.json
  releases/<release_id>/
    configuration.yaml
    elevators.yaml
    elevator_internal_poses.yaml
    validation.json
    manifest.json
    current.json
```

A valid draft is persisted with each resolved `map_asset_epoch` and
`map_asset_digest` stamped into its floor binding. If either identity component
changes after review, publish fails with `MAP_ASSET_EPOCH_CHANGED` or
`MAP_ASSET_DIGEST_CHANGED` until the App saves and reviews a new draft.
An invalid saved draft can never become publishable merely because maps are
later created or repaired: publish returns `DRAFT_REVIEW_REQUIRED`, and the
client must save/review a new revision so every digest is server-stamped.
Those digest fields are server-managed; an explicit rebind operation removes
the stale fields before saving so the server can stamp current values and
produce a new revision.
The commissioning document is bounded to 2 MiB, 16 elevators, 64 floors per
elevator, and 128 total floor bindings. Asset resolution is cached per
`(floor_id,map_id)` during one validation. Map manifests and Nav map YAML are
bounded before parsing; each Nav YAML `image` must be exactly one relative
root-level filename matching the digested `nav_map.pgm` (no parent
components). Root `image`, `resolution`, and `origin` must each occur exactly
once; map dimensions are read only from that authenticated PGM, never from a
nested or duplicate `image` key.
Despite the historical `configuration.yaml` filename, the commissioning
document and every persisted configuration/manifest/current selector use the
strict JSON subset documented by the App API. They are limited to 64 nesting
levels. YAML-only syntax, malformed UTF-8/Unicode escapes, trailing content,
and duplicate decoded object keys are rejected before `yaml-cpp` is called.
This includes escaped-equivalent keys such as `elevators` and
`\u0065levators`, and applies at every object level (including `floors`,
`map_asset_epoch`, and `map_asset_digest`). Stored duplicates are an integrity
failure, not a last-key-wins update. Integral scalars stay textual through
preflight so a valid positive `uint64` epoch is never rounded through
floating-point.
The reviewed configuration, including unknown extension fields and their JSON
scalar types, is retained in the release; generated runtime topology remains a
separate strictly validated projection.
The default GET reports `configuration_source: "draft"|"current"|null`;
when a draft exists it is returned for editing even if
`current_release_id` points at a different published release.

Existing schema-v1 immutable releases remain byte-preserved and readable for
audit and map-reference checks, but are marked `legacy_read_only=true` and
cannot be selected as rollback targets. An existing schema-v1 draft is exposed
by GET as a non-persisted schema-v2 migration preview with
`migration_required=true` and `source_schema_version=1`; GET never rewrites the
draft. Migration selects `landing` deterministically from the first present
legacy field in `hall_wait`, `doorway`, `exit` order. A present but malformed
higher-priority pose is retained as an invalid `landing` and never hidden by a
lower-priority fallback. PUT accepts a schema-v1 document only as migration
input, persists the result as a `draft-v2-*` revision, and all new publish or
rollback output is schema v2. Manifest and current-selector envelope schema
remain version 1 and are separate from the configuration-content schema. The
commissioning App upgrades editable v1/v2 content to an incomplete schema-v3
draft without inventing `cabin_panel` or either panel side; an operator must
explicitly supply those fields before v3 publication.

`map_asset_epoch` is a persistent positive `uint64` version allocated by
`robot_map_asset_identity` under
`maps_release/.map_asset_registry`. The registry is process-safe and globally
monotonic; identical current content is idempotent, while changed content or a
return to an older digest consumes a new epoch. Empty, corrupt, or regressed
state fails closed, and lookup is read-only. Every bind and lookup audits the
exact source-map manifest layout against the maximum committed epoch; an older
non-empty registry is rejected, while unrelated elevator/current manifests are
ignored. A missing read-only registry or lock is not created as a side effect.

Every committed source bundle is written as `njrh.map_manifest.v2` with the
same epoch, digest algorithm, digest contract, and digest. Manifest writes use
a same-directory temporary file, file fsync, atomic rename, and parent
directory fsync on Linux. The manifest itself and `poses.yaml` are excluded
from the content digest.

Pending activation-journal recovery preserves the same identity rule. A
complete `njrh.map_manifest.v2` source is verified against its recorded
epoch/digest before recovery can update `active` or recreate `current/`; source
drift keeps the journal, latches integrity degradation, and fails closed.
Only a genuinely legacy manifest with no v2 identity may be stamped during
the one-time startup migration.

`map_asset_digest` is a canonical `sha256:<64 lowercase hex>` wire identity.
The v1 digest sorts stable logical asset names and length-prefixes every name
and byte payload before hashing. The framing itself does not inject filesystem
roots; payload bytes are nevertheless hashed exactly, including paths already
recorded inside `asset_report.json`. The release manifest records
`asset_epoch`, `asset_digest_algorithm: "sha256"`, and
`asset_digest_contract: "njrh-map-asset-bundle-v1"`. The release's own historical
`configuration_digest_algorithm: "fnv1a64"` remains a separate legacy content
identity and must not be substituted for the map asset digest.

Once the current elevator release binds a floor/map, the HTTP keepout mutation
endpoint rejects in-place edits with `ELEVATOR_CONFIG_MAP_IN_USE`. This prevents
the immutable release from silently becoming stale. Commissioning must create
or select the intended map asset version, update/review the elevator draft, and
publish a new release instead of modifying a bound bundle in place.

`maps_release/<building_id>/elevators.yaml` and
`elevator_internal_poses.yaml` are stable symlink views through the same
authoritative `current` directory selector. On Linux, publishing writes and
fsyncs the immutable release first, then performs one atomic selector rename;
consumers that need topology and poses as a coherent pair must resolve/fix the
selected release once and read both files from that release directory.
Rollback copies a schema-v2 historical configuration into a new monotonic
release; it never rewinds or overwrites history. Schema-v1 releases are
read-only and must first be migrated through a reviewed v2 draft. A selector
fsync failure returns an explicit reconciliation error instead of claiming
success.

The three internal poses never enter a floor's ordinary `poses.yaml`.
`GET /maps/poses` and `/maps/semantic_layer` also hide any legacy
`type=elevator_internal` or reserved `eip_` records. Ordinary pose mutation
cannot create, replace, or delete them, and ordinary precheck/navigation plus
docking target/predock resolution rejects them with
`ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION`.

Every successful publish/rollback says:

```json
{"asset_published":true,"runtime_applied":false}
```

It does not switch a map, reload localization, start or stop Nav2, send a
goal, publish Twist, or release a safety hold. The future elevator mission
adapter must consume a selected release through
`robot_elevator_manager::load_elevator_release()`, its transaction-owned
operating mode, and a floor-switch transaction. Loader success is
configuration preflight only.
The authoritative epoch/digest identity is frozen in the returned source and
target plans, but the live floor/localizer/costmap evidence chain remains
disabled.

## Elevator test HTTP contract

### Production no-persistent-lock policy (2026-08-21)

The production `robot_api_server` constructs both the elevator execution
module and the ROS runtime adapter with persistent recovery locking disabled.
This is a code-level production policy, not an operator/YAML switch.

- A task failure still stops/cancels its owned work and enters automatic
  cleanup. Failed cleanup proof is retried automatically; it does not become
  terminal `LOCKED` after a retry budget. The automatic cleanup snapshot and
  its transient motion-admission fence do not globally reject unrelated API
  work while a recovery endpoint is offline.
- A retained historical journal is loaded as automatic restart cleanup and is
  released without App confirmation. It is not projected as
  `ELEVATOR_EXECUTION_RECOVERY_REQUIRED` to unrelated state-changing APIs.
- If the complete runtime restarts on a different confirmed map, restart-only
  cleanup accepts that healthy localizer/bridge identity as superseding the
  stale transaction after owned actions/resources and dual-odometry stop are
  proven. It conditionally releases only that transaction's owner hold instead
  of requiring the robot to switch back to the obsolete frozen map. Ordinary
  live failure cleanup remains exact-map gated.
- A prepare rejection or journal write failure is reported as `FAILED` rather
  than installing a persistent recovery latch. A rejected prepare also
  reopens the transient motion-admission fence.
- `POST /api/v1/elevator-test/recover` remains in the compatibility wire
  surface for old clients and forensic builds, but production operation does
  not require it and must not present an unlock button.

This policy removes the elevator-specific persistent lock only. An actually
running elevator transaction remains mutually exclusive with another
elevator transaction, and estop, watchdog, active safety holds,
`robot_safety`, and final `cmd_vel` arbitration remain authoritative. See
`../../docs/elevator_nonpersistent_recovery_policy.md`.

The commissioning App can discover all five endpoints through
`GET /api/v1/openapi`:

```text
POST /api/v1/elevator-test/start
GET  /api/v1/elevator-test/state?transaction_id=<id>
POST /api/v1/elevator-test/confirm
POST /api/v1/elevator-test/cancel
POST /api/v1/elevator-test/recover
```

### Automatic elevator arm button control (2026-08-25)

The production Jetson overlay enables `elevator_arm_button_control_enabled`
and binds the vehicle-facing client exclusively to `127.0.0.1:8083`. The App
continues to select `building_id`, `elevator_id`, `source_floor_id`,
`source_map_id`, `target_floor_id`, `target_map_id`, and
`expected_release_id` in the existing `start` transaction; it never calls the
arm black box directly.

At `hall_call` and `cabin_panel`, the ROS runtime executes one idempotent
sequence while the owner safety hold is active:

```text
POST /api/v1/arm/ready and poll its task
-> POST the button task and poll its task
-> POST /api/v1/arm/release and poll its task
```

The hall signal is derived from the frozen, live-validated source floor and the
selected target floor as numeric levels: target greater than source sends
`up`; target less than source sends `down`. `B1`, `B2`, and similar identifiers
are building IDs, never floor IDs, and are rejected by the floor parser. A
single elevator transaction pins one `building_id`; it cannot cross buildings.
If a building has basement maps, their floor IDs must use the explicit numeric
forms `-2` or `-1`. Equal numeric levels are not a valid elevator trip. Cabin
buttons accept only explicit `-2`, `-1`, or `F1` through `F20` (plus positive
numeric API equivalents). Transaction/effect-
derived mission IDs make retries idempotent. For feature validation, a task
completes when the black-box task endpoint reports `state=succeeded`; backend-
specific progress states and optional result metadata are not used as gates.

The deployed `/api/v1/elevator/call` capability performs the physical hall
button press and advances automatically when its task reports success. If a
compatible older backend explicitly returns `501 capability_unavailable`, the
existing `CALL_BUTTON_PRESSED` operator step remains available after the
release task and is never reported as a physical press. Source-door open,
target-floor arrival, and target-door open always remain operator gates.
Cabin `press-floor` failures are reported as functional failures. Runtime
prepare, normal hold release, and recovery hold release never call arm health
or status and are not blocked by arm state during this validation phase.

`start` first validates the exact building/elevator/source/target tuple through
`robot_elevator_manager::load_elevator_release()` and requires the App's exact
`expected_release_id`. The production Jetson overlay enables the ROS runtime
adapter. It owns the exact release binding, the remaining physical-observation
confirmation gates and the explicit hall-call fallback gate,
the Nav2 and atomic floor-switch actions, the operating-mode lease,
localization pause handoff, and the owner-scoped safety hold. Neither the API
nor the App publishes chassis velocity.

The first `HALL_CALL` navigation effect uses the ordinary hall-approach scope;
the elevator FSM no longer creates an execution session. Its post-hold-release
admission requires fresh interlock evidence
with no hold, no low-level/effective motion block, and no engaged execution
session or lease. A fresh `COMMAND_STALE` is accepted only as the expected
first-Nav2-command warmup; estop, localization, dock-contact, retained hold, and
retained/foreign execution-session evidence remain hard blockers. All later
elevator navigation intents use the separate execution scope and require the
exact runtime owner, mission, operating-mode lease ID, operating-mode contract,
and normal Nav2 source. They do not create a safety execution lease. A fresh
`motion_allowed=true` sample
cannot bypass either scope's interlock proof.

Every elevator-scoped Nav2 action also owns one immutable controller-session
identity. The runtime maps the selected behavior-tree profile to its exact
`FollowPath` plugin, sends `BEGIN(transaction_id:effect_sequence)` before goal
submission, and sends a matching `END` on every return path. A new identity
authoritatively clears stale terminal timeout/failure/route state even when the
previous `END` response was lost; a stale `END` is rejected. Periodic replans
inside the same action keep the identity and do not restart motion. Ordinary
Nav2 goals never call these private controller services.

Elevator-test no longer calls `/safety/set_execution_lease`; the deprecated FSM
effects are rejected if an old caller tries to execute them. One operating-mode
worker renews the exact owner/mission mode independently from Nav2 and
`FloorSwitch`. A transient failure can be recovered by reapplying that exact
mode and restarting the worker; an expired retired mode ID is replaced only
after the mode service proves the old lease absent. While the owner hold is
active, renewal failure does not fail health polling and neither
`begin_floor_switch` nor `await_floor_switch` contains a renewal-driven cancel
path. The next motion effect must renew/reacquire mode before releasing hold.

The correction-pause handoff is transaction scoped and level triggered. The
API accepts only feedback for the active FloorSwitch submission, rejects stale
or out-of-order effect sequences, and monotonically latches
`caller_pause_handoff_ready`. A following `VERIFY_PAUSE_HANDOFF` sample cannot
erase readiness; the exact transaction's reliable floor-status stage is also
accepted as a missed-feedback reconciliation source. See
`docs/floor_switch_pause_handoff.md`.

For schema-v2 motion effects, `hall_call` and `source_landing` keep the yaw stored in the
frozen configured pose. `enter_cabin` keeps the cabin pose's configured final
yaw; the completed source-landing step supplies the live ingress yaw to the
scoped planner. A cabin target may be laterally offset inside the car, so the
landing-to-cabin chord is neither a valid doorway normal nor a final-pose
heading. `target_landing` continues to
derive the directed cabin-to-landing exit heading. `hall_call` uses the API
server's existing freshness-gated map pose: at more than 2.5 m, or without a
fresh finite pose, it leaves `NavigateToPose.behavior_tree` empty and uses
ordinary Nav2; at 2.5 m or less it selects the fixed installed scoped tree.
This lets a nearby elevator-owned goal translate/reverse to a clear staging
pose instead of failing an obstructed startup spin. After that hall goal,
`source_landing` keeps the scoped tree and its commissioned motion order, but
uses an unchecked bounded path, disables controller clearance/revision, and
refreshes the transaction collision-monitor bypass. `enter_cabin` and
`target_landing` select the fixed installed
`navigate_elevator_cabin_entry_direct.xml`; that tree selects only
`ElevatorCabinEntryDirect` and `ElevatorCabinEntryDirectFollowPath`. The App
cannot submit that planner/controller choice.

Schema-v3 source `hall_call -> landing` uses the installed
`navigate_elevator_reverse_entry_staging.xml` tree and the dedicated
`ElevatorReverseEntryStagingScoped`/
`ElevatorReverseEntryStagingFollowPath` IDs. Its fixed sequence remains goal
yaw first, lateral motion second, and forward/reverse closure last, while its
planner, controller and command route now ignore obstacles exactly like the
later cabin-transit effects. Every later schema-v3 cabin-transit effect selects
`navigate_elevator_cabin_entry_direct.xml`: `landing -> cabin`,
`cabin -> cabin_panel`, target `cabin_panel -> cabin`, and `cabin -> landing`.
The bounded direct planner does not reject costmap cells, the controller does
not run command-clearance checks or replans, and the exact transaction routes
`/cmd_vel_nav` around `collision_monitor` into `robot_safety`. The final safety
arbiter, exact DOORWAY mode, estop, localization, watchdog, speed,
reverse, and lateral gates remain mandatory. The pre-call `hall_call` approach
alone remains on the obstacle-checked ordinary/nearby-hall chain.

Runtime `prepare` failures are cleanup-required by default. Only an explicit
adapter result proving that rejection happened before every effect may
terminate directly as `FAILED / RUNTIME_PREFLIGHT`. The transaction layer does
not infer zero side effects from failure-code text.

Ordinary navigation, confirmation, doorway, ride, floor-transition, and cancel
failures are retryable task failures. Through `SWITCHING_FLOOR`, cleanup uses
the exact journaled source-floor runtime identity; after SwitchFloor succeeds,
it uses the exact target-floor identity. The adapter cancels owned effects,
reconciles runtime resources, releases the transaction hold, and terminates as
`FAILED` or `CANCELLED`. The structured
`motion_not_authorized_proven` cabin-entry field remains diagnostic but is no
longer required to avoid a persistent task lock.

Every state response exposes the recovery evidence explicitly:

```json
{
  "state": "FAILED",
  "safety_hold_state_known": true,
  "safety_hold_active": false,
  "dual_odom_stop_proven": true,
  "runtime_resources_reconciled": true,
  "failure_origin_state": "NAVIGATING_TARGET_LANDING",
  "failure_origin_effect_kind": "NAVIGATE_TO_POSE",
  "cleanup_disposition": "TARGET_OUTSIDE",
  "recovery_required": false
}
```

`safety_hold_active` is meaningful only when
`safety_hold_state_known=true`. An unknown terminal hold state remains
recovery-required; it is never interpreted as proven absence. A terminal
transaction that applied runtime effects also remains recovery-required until
`runtime_resources_reconciled=true`.

An unfinished journal is never resumed as an elevator mission after process
restart. It resumes only cleanup: the adapter obtains a monotonically sequenced
hold, cancels unknown Nav2 action state, and consults the journaled action
boundary before touching FloorSwitch. `BEGIN_FLOOR_TRANSITION` and
`SWITCH_FLOOR` runtime intents mean the floor action may have been submitted,
so its endpoint, cancel response, and terminal evidence remain mandatory. A
proven earlier phase such as `NAVIGATING_HALL_CALL` skips that unrelated action
plane. Unknown phase evidence remains fail-closed. The adapter then proves
fresh wheel/local-odometry settle and reconciles the
exact source/target context and owned resources, then conditionally releases
the owner hold. For a phase that proves FloorSwitch was never submitted, a
complete-runtime restart may already have rebound navigation to the other
immutable endpoint in the same frozen release. Cleanup then accepts the
currently confirmed source or target endpoint only when map epoch/digest,
localizer, bridge, unique TF, goal-start safety, resource absence, and
dual-odometry stop all agree. It never accepts an unrelated floor, and any
possible FloorSwitch submission still requires the recorded outside side.
A current-schema historical normal-stage
`ELEVATOR_EXECUTION_ON_SITE_SERVICE_REQUIRED` snapshot is automatically
reclassified from `RETAIN_LOCK` to the side exactly matching its journaled
current floor/map and follows this ordinary cleanup path. While cleanup is in
progress, unrelated state-changing HTTP APIs remain blocked; after terminal
`FAILED/CANCELLED` with hold absent and resources reconciled, admission reopens
and a new test may start.

A bounded cancel wait does not discard an already-submitted recovery request.
The adapter retains exactly one Nav2 or FloorSwitch cancel-all future across
startup recovery attempts, reports `*_CANCEL_RESPONSE_PENDING`, and observes
the late response without submitting a duplicate request. Only an exception or
other loss of the retained future is classified as `*_RESPONSE_UNKNOWN` and
requires a complete runtime restart. No pending response is treated as cancel
success, action-idle proof, or authority to release the owner hold.

The production supervisor restarts the complete runtime chain rather than the
API process alone. If a previous cold start exhausted the bounded endpoint
readiness window and persisted
`ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN`, the next full-chain cold start
does not wait for those replacement endpoints a second time. It finalizes only
a schema-v3 terminal ordinary-stage lock with an exact current source/target
floor-map identity and matching `SOURCE_OUTSIDE`/`TARGET_OUTSIDE` disposition.
The historical failure code is retained, the transition is audited as
`NORMAL_STAGE_LOCK_CLEARED_ON_COLD_START`, and no motion or dual-odometry proof
is invented. Maintenance, prepare, retained-lock, identity-mismatch, journal,
and storage/audit failures are excluded.

`POST /api/v1/elevator-test/recover` remains a maintenance transaction for
corrupt, unsupported, or genuinely unreconciled historical evidence, not a
normal cancel/retry path. Its body must exactly match `transaction_id`,
`expected_state=LOCKED`, and `effect_sequence`, and must include a non-empty
`operator_id` and `reason`. Existing strict historical recovery shapes remain
readable for compatibility, but new ordinary stage failures do not require
this endpoint.

The single request runs asynchronously. The backend journals intent before
effects, then reports `phase=RECOVERY_VERIFYING` while it acquires a
transaction-scoped recovery hold, cancels unknown Nav2/FloorSwitch actions,
proves Nav2, WebSocket teleop, mapping and docking idle, proves there is no
execution session, operating-mode lease or localization pause, and proves
fresh wheel/local odometry settled. It journals the next intent before
`phase=RECOVERY_RELEASE_PENDING`, durably creates a sequence-scoped immutable
release-pending audit, runs the final action cancel barrier immediately before
release, then calls the dedicated robot-safety compare-and-release endpoint.
That endpoint accepts only the exact transaction hold, a strictly newer
command sequence, the generation from the final fresh resource-absence sample,
and an interlock state with both execution session and lease absent. It checks
and removes the hold in one arbiter critical section. The response must again
prove the exact sequence, session/lease absence, one-step generation advance,
and owner-hold absence before an immutable completion audit and atomic
`FAILED / RECOVERY_COMPLETE` persistence.
The public `state` remains `LOCKED` throughout both recovery phases.

Maintenance recovery preserves its journal-first audit and exact identity
checks. HTTP admission and WebSocket teleop share the same execution interlock,
so neither can bypass an active cleanup or maintenance recovery. Recovery never
deletes, renames, or edits `maps_release/.elevator_test/active.yaml` out of
band.

Normal `HOLD_AND_CANCEL` is phase-aware. `SOURCE_OUTSIDE` and
`TARGET_OUTSIDE` resolve exact Nav2/floor action terminal state and global
runtime idle, release and prove absence of owned execution/mode/correction/
floor-handoff resources, validate the exact live floor identity, and prove
fresh wheel/local-odometry stop. The backend persists
`CLEANUP_RELEASE_PENDING` plus its immutable audit before the
generation-conditional owner-hold release. Success automatically ends
`FAILED`/`CANCELLED`, clears the runtime binding, and reopens the fence; the App
does not call `recover` for an ordinary failure.

The FloorSwitch Nav2 action evidence is event state, not a heartbeat. The
latest active/terminal status is latched until the next action event, while the
0.75 second evidence window continues to apply to live hold and wheel/local
odometry samples. Manual confirmation latency therefore cannot expire an
already terminal Nav2 goal.

Cleanup identity uses `/global_localization/asset_state` as a latched,
transient-local asset identity snapshot. Its receipt age is not a heartbeat and
does not expire an otherwise exact identity. `/localization/floor_health` and
`/localization/bridge_status` are the live readiness proof and must both remain
within `state_evidence_max_age_sec`, exact for the outside floor, and settled.
An identity-proof failure reports asset presence, both live evidence ages, and
the localizer/health/bridge exact and freshness booleans.

Pending audit replay is byte-strict. A restart that finds the pending journal
but no audit creates it only after exact rebind/reproof; a conflicting audit
prevents physical release. Completion audit replay compares a stable safety
and frozen-identity subset (checkpoint, transaction/sequence, terminal state,
hold absence, dual stop, resource reconciliation, failure origin, current
floor, and complete pinned release identity) while ignoring only timestamps,
records, and non-safety detail. This keeps the crash window after physical
release but before final journal persistence idempotent.

If `api_token` is empty, `operator_id` is an audit label only and does not
authenticate a person. In that configuration the recovery endpoint accepts
only a loopback maintenance connection; LAN clients cannot release the hold.
With a configured token, a remote recovery request must pass the normal
`X-Robot-Token` check. A commercial deployment must enable authentication and
populate the operator identity from the authenticated principal.
The installed runtime supplies `ROBOT_API_TOKEN` through the process
environment; the node uses it only when the explicit ROS parameter is empty.
Startup scripts must never expand the secret into a ROS argument or shell
command string, so ordinary process listings do not expose the credential.

An unreadable, symlinked, unsupported, or durability-failed active journal is
not equivalent to “no transaction.” The module exposes a journal recovery
interlock even when it cannot render a snapshot, and the shared HTTP/WebSocket
policy blocks all unrelated state-changing calls. A legacy journal that omits
`runtime_applied` is conservatively treated as possibly applied and therefore
requires on-site service.

## P6 floor-runtime negative interlock

The API permanently subscribes to:

- `/floor_manager/transition_status`
  (`robot_interfaces/msg/FloorSwitchStatus`);
- `/localization/floor_health`
  (`robot_interfaces/msg/LocalizationHealth`).

This is deliberately a negative-only compatibility fence. Before a live P6
transaction is deployed, missing typed messages and
`LEGACY_CONTEXT_UNSCOPED` do not grant or revoke ordinary runtime readiness.
Explicit `transition_active`, invalid runtime context, mutation-stage
`FloorSwitchStatus`, or `FAILED_LOCKED` blocks new navigation, mapping start or
save, map/pose/keepout mutation, floor selection, manual localization, docking,
undocking, and safety resume. Safety stop plus navigation/mapping/docking
cancel/stop remain available so an operator can converge to a safe state.

The current preflight-only floor Action reports `PREFLIGHT` then `BLOCKED`
without changing assets. Those two states do not latch the API interlock.
`FAILED_LOCKED` is sticky across an inconsistent healthy bridge sample. HTTP
rejections use top-level code `FLOOR_TRANSITION_BLOCKED`, with a specific
`reason_code` and transaction ID. `/api/v1/status` exposes
`floor_runtime_interlock`.

Entry checks are repeated immediately before the main background commits
(Nav2 goal send, mapping process launch, localization trigger, docking/fine
docking, undock service, and asset writes). This narrows races but is not a
distributed execution lease. A completed transaction occurring between target
resolution and a second check requires the existing transaction/mode/floor
identity fences; this ordinary API interlock alone does not authorize elevator
or floor-switch motion.

Jetson-isolated verification is part of the package tests:

```bash
colcon test --packages-select robot_api_server
```

`floor_runtime_http_smoke` uses its own ROS domain and temporary API process. It
verifies thirteen blocked positive-action endpoints, permits safety stop,
confirms a non-mutating preflight failure does not latch, and confirms a failed
lock cannot be hidden by a healthy sample. It also creates an isolated
`/navigate_to_pose` ActionServer so `navigation.active=true` is real, then
proves exact same-runtime-map selection is a zero-file/zero-service-call no-op
while a different map remains `FLOOR_SELECTION_RUNTIME_BUSY`. The smoke test
also kills a starting runtime owner and requires the next exact start to launch
again instead of reusing the stale context. A separate two-start fixture stamps
a legacy map to v2, tampers one authenticated asset behind a pending activation
journal, and proves restart leaves the journal/source untouched with integrity
degradation latched. Every spawned test server overrides
`last_navigation_map_file` into its temporary fixture root, so running CTest
cannot replace the production autostart selection. Its two-map PNG fixture
also proves that an exact saved-preview request returns the requested
immutable map even when another preview is newer, and that unknown IDs,
missing exact PNGs, and building/floor mismatches fail closed.
- `POST /api/v1/maps/poses` upserts one semantic delivery point into backend map assets.
- `PUT /api/v1/maps/poses/{pose_id}` updates one semantic delivery point by stable ID.
- `DELETE /api/v1/maps/poses/{pose_id}` deletes one semantic delivery point by stable ID.
- `PUT /api/v1/maps/poses/batch` replaces the full point set for backend admin or batch import.
- `POST /api/v1/maps/poses/save` upserts semantic delivery points into the selected `maps/<map_id>/poses.yaml` and synchronizes `current/poses.yaml` when that map is active.
- `POST /api/v1/maps/poses/save_current` writes a semantic point using the same fresh `map -> base_link` pose as `/api/v1/robot/pose`, so the App does not convert pixels to metric coordinates for live marking.
- `GET /api/v1/maps/filters/keepout` reads the keepout mask asset metadata plus App-authored keepout semantic JSON.
- `POST /api/v1/maps/filters/keepout/save` replaces the complete App-authored keepout layer. It generates the semantic JSON and matching Nav2 mask YAML/PGM as one rollback-capable transaction, synchronizes active-map projections, and hot-loads the mask only when the exact runtime map is selected.
- `GET /api/v1/navigation/pre_goal_check` returns the read-only dock/contact gate that will be used before a normal point navigation goal. It resolves `pose_id` from `poses.yaml` when provided, checks direct `x/y/yaw` map-frame goals when provided, reports `/navigate_to_pose` action-server admission readiness, and never calls `/docking/undock` or sends a Nav2 action.
- `POST /api/v1/navigation/goal` resolves a saved pose or direct map-frame pose, creates a `navigation_goal` job, and returns `202` quickly before waiting for controlled undock, post-undock relocalization, bridge readiness, or the Nav2 action goal handle. The background job then performs any required `/docking/undock`, waits for post-undock localization/settle readiness, waits for `robot_localization_bridge.safe_for_goal_start`, sends the `NavigateToPose` goal to `/navigate_to_pose`, and records `pre_navigation_undocking`, `waiting_for_goal_start_readiness`, `sending_nav2_goal`, and failure diagnostics for App polling. Normal goal admission does not synchronously poll Nav2 lifecycle `GetState` services or wait for the action goal response; those probes are startup/diagnostic checks because they can time out while the controller-hosted costmaps and planner are actually active under Jetson/FastDDS load. Success responses include the dock snapshot plus `pre_navigation_undock` / `pre_navigation_undock_detail`; later stage failures are reported in `/api/v1/navigation/state`.
- Dock/contact detection for navigation admission is explicit, not position-based. In addition to stable BMS contact and `/docking/status`, the server reads `docking_contact_latch.json`, written by charging-session evidence, docking success, manual maintenance confirmation, and undock success. `pre_navigation_dock_check.dock_contact_snapshot`, `dock_contact_latch_source_strength`, `charging_session_latched`, `dock_occupancy_state`, `dock_occupancy_evidence`, `strong_live_docked`, `latch_valid_for_auto_undock`, `docked_state_class`, `docked_evidence`, and `docked_warnings` expose this state. A stale legacy `source=bms` latch is weak safety memory and can be cleared by stable BMS no-contact plus no live docked/charging/undocking context. New charging evidence is stored as strong `source=charging_session`; restart-time idle/no-contact is not enough to clear it. A `source=charging_session` latch is auto-cleared only after confirmed live undock plus stable BMS no-contact, or by explicit maintenance/session clear, so full-charge BMS idle (`current=0`, `present=false`, status unknown) does not let normal Nav2 skip controlled undock when the robot is still physically on the dock.
- `POST /api/v1/navigation/cancel` accepts a background cancel job, publishes zero velocity immediately, uses a short action-server probe for responsive cancel behavior, then cancels the active Nav2 goal while keeping the resident Nav2/localization runtime alive by default. Use `POST /api/v1/navigation/stop` or `/api/v1/navigation/stop_runtime` only for diagnostics or recovery when the runtime must be torn down.
- `GET /api/v1/navigation/state` returns the latest background navigation cancel job and `navigation_goal` state for App polling and diagnostics. Normal point goals default to `goal_completion_policy=pose_required`; the API sends x/y/yaw to Nav2 and treats the Nav2 action result as input to commercial final verification, not business completion by itself. The API waits for bridge `map->odom` smoothing before the final pose read, requires 0.06 m XY and 0.05 rad yaw for normal completion, retries the same Nav2 goal for bounded light/recovery overruns, and enters `degraded` instead of reporting false `task_complete=true` when final verification remains outside the commercial gate. The ordinary terminal correction has a separate lateral hysteresis contract: body-frame lateral error above 0.04 m enters correction, an engaged correction cannot complete until lateral error is at most 0.03 m, and success is rechecked only after `/wheel/odom` is physically stable and Ranger feedback has returned to aligned DUAL_ACKERMAN mode. One bounded correction is allowed if the settled pose leaves the strict gate. If only yaw is outside the yaw-alignable XY window, one bounded ordinary API `final_yaw_align` fallback may run through `/cmd_vel_api`; that fallback also waits for bridge smoothing and pauses `robot_localization_bridge` global-correction intake until the final spin exits. `position_only` is an explicit engineering opt-out.
- `POST /api/v1/docking/start` resolves a saved dock contact pose, prefers a manual pre-dock pose (`predock_pose_id`, `approach_pose_id`, or the `dock_id_predock` naming convention), falls back to a geometric pre-dock offset only when no manual point exists, checks bridge `safe_for_goal_start`, and sends Nav2 toward that pose. The dedicated stable-path BT selects the ordinary Nav2 `goal_checker`, so Nav2 must natively satisfy both `0.06 m` XY and `0.05 rad` yaw at the commissioned predock pose. With `docking_predock_early_handoff_enabled=false`, only that terminal result followed by wheel/local-odometry stop proof can reach handoff. After `FINE_DOCKING_BRIDGE_SETTLE` and correction pause, the production default `docking_delegate_staging_motion_to_manager=true` records the residual evidence and calls `/docking/start` without acquiring the docking command owner, changing Ranger mode, or publishing yaw/lateral velocity. `robot_docking_manager` then owns camera-guided capture, combined forward/lateral PARALLEL approach, final insertion, BMS stop, and bounded retry as one state machine. The previous API physical staging servo remains an explicit rollback-only false setting. When fine docking reports success, failure, or stop, the API can trigger a configured localization pass before returning the final docking state so Nav2 does not resume from pose accumulated only through non-Nav2 motion.
- `/docking/status` is parsed as one leading state/event code followed by diagnostic `key=value` fields. Only the leading code can complete or fail the API docking job. In particular, `contact_stopping ... contact_stop_feedback_timeout=false|true` remains a running zero-command/brake-confirmation state; the diagnostic timeout flag cannot be reclassified as `FINE_DOCKING_TIMEOUT`. Explicit leading codes such as `contact_verify_timeout`, `contact_verify_failed_*`, `dock_feature_not_found`, `request_rejected*`, and `undock_failed_*` remain terminal failures.
- Successful docking releases the bridge correction pause through a dedicated deferred-work queue. `/docking/status` is consumed by a `SingleThreadedExecutor`, so that subscription callback must never synchronously wait for `/robot_localization_bridge/set_correction_paused`; doing so prevents the same executor from completing the service response and falsely creates `DELAYED_SIDE_EFFECT_UNKNOWN`. The generic service failure bound remains `service_timeout_sec=8.0`; it is separate from the physical contact confirmation window and is not extended to mask executor starvation.
- Predock yaw and lateral predicates remain separate read-only handoff evidence fields, but under the production delegation switch they are not API-owned physical stages. Nav2 has already satisfied the commissioned pose's normal `0.06 m / 0.05 rad` gate before these fields are evaluated; the manager's later camera alignment is a distinct fine-docking coordinate contract.
- `POST /api/v1/docking/undock` accepts the App's undock intent only when the robot is docked, live charging contact is detected/inferred from BMS status, or the explicit dock-contact latch indicates a manually confirmed docked state, then calls `/docking/undock`; after odometry-confirmed departure it calls `/global_localization/trigger`, whose wrapper alone arms the bridge and dispatches Isaac, then waits for `map -> base_link` to reflect the accepted result. The App must not publish reverse velocity directly.
- `POST /api/v1/docking/confirm_docked` and `POST /api/v1/docking/clear_docked_latch` are maintenance recovery endpoints. They only set or clear the persistent docked latch and never send velocity.
- `POST /api/v1/docking/cancel` / `POST /api/v1/docking/stop` cancels the pre-dock Nav2 goal, publishes zero velocity, and calls `/docking/stop` using the configured docking stop service wait.
- `POST /api/v1/navigation/goal` uses the resident runtime context as the immediate admission contract and defers `robot_localization_bridge.safe_for_goal_start` plus AMCL correction readiness to the accepted background job. Confirmed same-map runtime context plus a fresh `map -> base_link` pose skips `/global_localization/trigger`; if bridge/AMCL readiness is still transitioning, the job waits briefly and then fails with `failed_goal_start_readiness` instead of blocking the mobile HTTP request. Explicit localization recovery remains a separate endpoint for cold or unconfirmed runtime context, stale/missing map-frame pose, controlled undock, docking transitions, or explicit `force_relocalize`. If the Nav2 action server is unavailable after admission, the accepted background `navigation_goal` is marked failed instead of blocking the mobile HTTP request.
- `GET /api/v1/docking/state` returns the latest docking job, `/docking/status`, `charging_contact`, `inferred_docked`, `can_auto_undock`, and the same `pre_navigation_dock_check` used by normal point navigation.
- `GET /api/v1/status` includes HTTP active/max connection counters; the server rejects excess clients with `503` instead of spawning unbounded detached threads.
- `infrastructure/http/api_gateway_module` is the single owner of public HTTP
  start/stop, `OPTIONS`, token validation, access/event logs, connection
  observations, and `/api/v1/openapi`. The composition root receives only
  authenticated normal requests and retains the established business-module
  precedence; WebSocket teleop keeps its socket/session and motion-admission
  behavior while reusing the same gateway token decision.
  `api_gateway_configuration_module` owns `host`, `port`, `api_token`, and
  `max_http_connections` declarations without changing their defaults.
- `application/subscriptions/subscription_module` owns
  `POST /api/v1/subscriptions/acquire|heartbeat|release`, the expiry timer,
  resource transitions, and the page-scoped `/scan` cache; the composition
  root only wires neighboring modules. Its companion configuration module owns
  all four subscription ROS parameters.
- WebSocket teleop keeps a permanent `/battery_state` guard and publishes zero velocity when charging/full or charge current is detected.
- Nav2 action calls have their own exception boundary, and the ROS callback executor is single-threaded so transient action-client faults are caught by the process-level spin loop instead of terminating the HTTP server.
- Other mapping and navigation start endpoints intentionally return `501` until formal ROS-native services/actions are exposed by `robot_mode_manager` or `robot_mission_manager`.
- The test web dashboard is not used as a backend dependency.

## Start

```bash
ros2 launch robot_api_server robot_api_server.launch.py
```

For phone clients, set a token before exposing the robot hotspot:

```bash
ros2 launch robot_api_server robot_api_server.launch.py \
  config_file:=/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/config/robot_api_server.yaml
```

## Minimal API

- `GET /api/v1/status`
- `GET /api/v1/robot/pose`
- `GET /api/v1/maps`
- `GET /api/v1/maps/semantic_layer`
- `GET /api/v1/maps/poses`
- `GET /api/v1/maps/filters/keepout`
- `GET /api/v1/mapping/2d/map`
- `GET /api/v1/openapi`
- `POST /api/v1/maps/poses`
- `PUT /api/v1/maps/poses/{pose_id}`
- `DELETE /api/v1/maps/poses/{pose_id}`
- `PUT /api/v1/maps/poses/batch`
- `POST /api/v1/subscriptions/acquire`
- `POST /api/v1/subscriptions/release`
- `POST /api/v1/subscriptions/heartbeat`
- `POST /api/v1/mapping/2d/start`
- `POST /api/v1/mapping/2d/stop`
- `POST /api/v1/mapping/2d/save`
- `POST /api/v1/mapping/stop`
- `POST /api/v1/mapping/save`
- `POST /api/v1/maps/delete`
- `POST /api/v1/maps/poses/save`
- `POST /api/v1/maps/poses/save_current`
- `POST /api/v1/maps/filters/keepout/save`
- `POST /api/v1/safety/stop`
- `POST /api/v1/safety/resume`
- `POST /api/v1/floors/switch`
- `POST /api/v1/localization/trigger`
- `POST /api/v1/navigation/start`
- `GET /api/v1/navigation/state`
- `GET /api/v1/navigation/pre_goal_check`
- `POST /api/v1/navigation/goal`
- `POST /api/v1/navigation/cancel`
- `POST /api/v1/navigation/stop`
- `POST /api/v1/navigation/stop_runtime`
- `GET /api/v1/docking/state`
- `POST /api/v1/docking/start`
- `POST /api/v1/docking/undock`
- `POST /api/v1/docking/cancel`
- `POST /api/v1/docking/stop`
- `WS /ws/v1/teleop`

`GET /api/v1/status` includes:

```json
{
  "mode": "IDLE",
  "state": "idle",
  "mapping_active": false,
  "navigation_active": false,
  "healthy": true,
  "message": "",
  "mapping": {
    "active": false,
    "state": "stopped",
    "map_topic": "/map",
    "map_endpoint": "/api/v1/mapping/2d/map"
  },
  "navigation": {
    "active": false,
    "state": "stopped",
    "action": "/navigate_to_pose"
  },
  "docking": {
    "active": false,
    "state": "stopped",
    "dock_id": "",
    "status_topic": "/docking/status",
    "last_status": ""
  },
  "bms": {
    "soc": 48.0,
    "soc_valid": true,
    "source_topic": "/battery_state",
    "age_sec": 0.02,
    "voltage": 491.0,
    "current": -0.7,
    "temperature": 18.9
  }
}
```

The App should treat `mode`, `state`, `mapping_active`, `navigation_active`, `healthy`, and `message` as the lightweight business-state contract. These fields are maintained atomically by `application/runtime_mode/` rather than by asking the App to inspect ROS nodes or topics directly. Its snapshot priority remains `ERROR > DOCKING > MAPPING_2D > NAVIGATION > IDLE`; docking admission and completion commit their related fields under one lock. Status and navigation-state handlers must not synchronously probe Nav2 lifecycle services on every mobile poll; blocking lifecycle checks belong to resume/navigation admission and explicit readiness diagnostics. The socket server uses a fixed worker pool (`max_http_connections`, default `16`) instead of detached per-request threads. If the server returns `503 {"error":"server busy"}`, the App should back off and retry instead of opening additional parallel status requests.

AMCL continuous-localization readiness is exposed from the runtime status file and bridge status, not inferred from old logs. `GET /api/v1/status` and `GET /api/v1/navigation/state` include `amcl_state`, `amcl_ready`, `amcl_degraded`, `amcl_degraded_reason`, `amcl_process_alive`, `amcl_scan_admission_alive`, `/amcl_pose` publisher count, scan-admission status publisher count, `amcl_correction_ready`, `amcl_correction_pending`, `localization_degraded`, and `using_triggered_baseline_only`. In `shadow` mode an AMCL startup/readiness failure can continue as a visible degraded Isaac-triggered baseline; in `gated` mode the runtime reports a readiness failure instead of silently treating AMCL as active. A stationary, seeded AMCL with no fresh correction yet is `amcl_correction_pending=true`; it is not a localization recovery requirement by itself, and clean no-motion static standby does not block goal start. Non-standby pending/not-ready correction is treated as a transition; accepted background navigation jobs wait briefly for correction readiness before sending Nav2, then fail as `failed_goal_start_readiness` if it does not recover.

Explicit business relocalization has a second gate after bridge acceptance. `robot_api_server` reads `/localization/bridge_status.last_explicit_relocalization_sequence` and waits for the expected sequence, `map -> odom` owner/freshness, `odom -> base_link` freshness, the static `base_link -> lidar_level_link` transform, at least two `/local_costmap/costmap` updates, and no new local-costmap MessageFilter drops before it sends the next Nav2 goal or starts GS2 fine docking. This is a settle barrier, not a TF tolerance increase. For post-undock goal release, the TF/bridge checks remain hard gates, while local-costmap update/drop and AMCL scan-admission transient checks are recorded as warnings so a successful undock and accepted relocalization do not discard the original navigation goal. Failures are reported as `POST_RELOCALIZATION_*` or `CANCELLED_BY_APP`, and `/api/v1/status` plus `/api/v1/navigation/state` expose `post_relocalization_settle`.

`soc` is normalized to `0..100`. Ranger currently publishes `/battery_state.percentage` as a percent value; if a future driver follows the ROS convention `0.0..1.0`, the API converts it to percent.

## Subscription Module

High-rate or page-specific ROS inputs are not kept subscribed permanently. The App should acquire resources when entering a page, heartbeat every 3-5 seconds, and release them on exit:

```json
{"client_id":"app_device_001","resources":["status","live_map","tf"],"ttl_ms":10000}
```

For compatibility with older App/Web builds, the server also accepts `clientId`, `lease_id`, `leaseId`, `subscription_id`, and `subscriptionId`. If no client identifier is provided, the request uses the compatibility lease `http:compat-default` instead of returning HTTP 400. Heartbeats may omit `resources`; the server refreshes the client's existing resources, or returns a successful no-op if none are currently leased.

Resources:

- `status`: compatibility resource for status pages. Safety state itself (`/safety/status` and `/safety/motion_allowed`) is subscribed for the whole API process with transient-local QoS, so releasing a page lease does not clear the safety cache used by `/api/v1/status` or motion admission.
- `live_map`: live `slam_toolbox` `/map` PNG rendering permission; the API also keeps an internal `/map` cache while 2D mapping is active for status readiness and save
- `scan`: `/scan` cache hook for laser-layer views
- `tf`: `/tf` cache for `map -> odom -> base_link` pose
- `teleop`: WebSocket mapping teleop lifecycle

When a resource refcount changes `0 -> 1`, page-specific ROS subscriptions are created. When it changes `1 -> 0`, page-only subscriptions are reset and high-frequency cache is cleared. Core health subscriptions and the `/map` subscription retained during active 2D mapping are not page-owned, so startup readiness and save do not depend on App polling order. Leases expire automatically after `ttl_ms`; default is `10000`.

Server-side ownership is consolidated behind
`application/subscriptions/subscription_module`. It owns HTTP dispatch,
expiration, transition semantics, and the `/scan` cache. `subscription_manager`
and `subscription_api` are internal collaborators; no subscription route or
resource-name switch remains in `robot_api_server_node.cpp`.

`GET /api/v1/mapping/2d/map` only serves live `/map` when `live_map` is currently acquired. Saved map preview does not require `live_map`. App map editors should request the immutable bundle directly with `?source=saved&map_id=<map_id>&building_id=<building_id>&floor_id=<floor_id>`; this reads that manifest's `localizer/<safe_map_name>.png`. An unknown map or missing exact PNG returns `404`, a building/floor ownership mismatch returns `400`, and an exact request never falls back to a newer runtime PNG or another floor's `current/` projection. `?name=<map>` and selector-free `?source=saved` remain legacy runtime-preview forms.

## Floor Selection, Navigation Start, And Disabled Live Switch

App editor-map selection is local UI state and must not call this endpoint.
Saved map previews and overlays are addressed directly by `map_id`.

`POST /api/v1/floors/switch` with `resume_navigation=false` is normally an
offline, selection-only asset request to `/floor_manager/switch_floor`. The
gateway resolves and verifies one immutable manifest, sends its exact
`map_id/asset_epoch/asset_digest`, and rejects
`FLOOR_SELECTION_IDENTITY_UNPROVEN` if the service does not echo the same
identity and source paths. The same `MapAssetCommitTransaction` remains held
from the initial snapshot through the service result and the activation
commit. The gateway re-reads and re-verifies the exact epoch/digest after
service success; any drift returns `FLOOR_SELECTION_SOURCE_DRIFT` without
changing `current/`. Only then may it atomically activate the map and build
the fixed `current/` projection. A
different map is rejected with `FLOOR_SELECTION_RUNTIME_BUSY` unless
navigation, mapping, docking, API goal jobs, Nav2 action goals, and the
navigation process are all stopped. On a successful selection it clears the
stale runtime-map context before projecting the selected manifest into
`current/`. It does not reload localization or start Nav2.

For ordinary navigation, the client next calls:

```http
POST /api/v1/navigation/start
Content-Type: application/json

{"building_id":"B1","floor_id":"F1","map_id":"map_..."}
```

The endpoint accepts only the exact single active map selected into the
backend-owned `current/` projection. It validates safe IDs, complete immutable
assets, the canonical server-side identity snapshot, matching
`building_id/floor_id/map_id/asset_epoch/asset_digest`, all fixed projection
files, and runtime exclusivity. A first accepted launch returns `202
navigation_resume_starting`; an identical fresh in-progress request returns
`202 navigation_runtime_starting_reused` without a second process. A request
for an unselected map returns `409 NAVIGATION_MAP_NOT_SELECTED`.

After either `202`, the caller must poll `GET /api/v1/navigation/state`.
Startup is complete only when navigation is active and healthy,
`safe_for_goal_start=true`, the runtime context is confirmed and `ready`, and
its exact building/floor/map identity matches the request. Until then the App
must not display the selected map as the current navigation map or submit a
goal.

The API child marks this shared runtime launch with
`NJRH_NAVIGATION_START_SOURCE=api_resume`. Unlike the systemd boot path, an App
resume first requires three consecutive advancing `/local_state/odometry` and
`odom -> base_link` observations before it starts selected-floor localization.
This prevents a just-stopped or graph-recovering local-state boundary from
producing `ODOM_BASE_TF_NOT_FRESH` after localization has already been spawned.
Failure is reported as `LOCAL_STATE_ODOM_TF_NOT_STABLE` through the normal
runtime context and navigation-state polling path. The check is sample-based,
not a fixed sleep, and does not change boot autostart ordering.

For compatibility with older App builds, an exact request for the confirmed
`ready` navigation runtime map is an idempotent no-op even while navigation is
resident. It returns `state=runtime_map_already_selected`,
`already_active=true`, `runtime_unchanged=true`, and
`selection_performed=false`. This branch validates the exact
`building_id/floor_id/map_id` and required assets, then returns before the
legacy floor service, runtime-context clear, manifest activation, or any
process operation. A different, missing, pending, or failed runtime identity
continues to return the normal busy response.

`resume_navigation=true` is now rejected with
`LIVE_FLOOR_SWITCH_DISABLED` before map lookup, active-manifest mutation, ROS
service calls, or process launch. The previous direct path to:

```text
scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh <building_id> <floor_id>
```

was removed because it bypassed the transaction Action and could update the
selected map before live switch failure was known. The script remains a
repository-owned single-floor startup implementation used by controlled
startup/recovery paths; it is not a valid cross-floor commit protocol.
The dedicated `/api/v1/floor-switch/start|state|cancel` endpoints now wrap the
strict Action that owns exact asset identity, bridge fencing, localizer reload
proof, target localization, fresh costmaps, and final readiness under a motion
hold. They do not re-enable this legacy flag.

The elevator runtime adds one post-Action handoff barrier before sending the
next goal: target-floor status must report AMCL seeded/tracking-ready (a valid
stationary static standby is sufficient), then at least two new, fresh and
stable target-map robot-pose samples must be observed. This barrier does not
compare source/target `cabin_panel` coordinates and does not transfer the old
map pose into the new map. The following `cabin` NavigateToPose action contains
only the target; Nav2 computes its start from the live target-map pose. After
the owner hold is released, `/safety/status=COMMAND_STALE` with the exact clear
elevator execution interlock is treated as first-command warmup, while estop,
invalid localization/mode/source, retained hold, or dock contact remain hard
fail-closed blockers.

## WebSocket Teleop

`/ws/v1/teleop` is for App-driven low-speed mapping teleop only. It is accepted only while the 2D mapping chain is active by default. It publishes `geometry_msgs/Twist` to `/cmd_vel_api`, so the command still flows through `robot_safety`, final `/cmd_vel`, and the `ranger_base` chassis core.

Teleop admission uses the same elevator-execution interlock as HTTP writes.
An elevator `LOCKED` state or either recovery phase rejects a new socket
session, even when no safety hold is currently visible.

Client messages:

```json
{"type":"cmd_vel","linear_x":0.20,"angular_z":0.10}
{"type":"cmd_vel","linear_x":-0.12,"angular_z":0.00}
{"type":"stop"}
```

Limits are enforced by server parameters:

- `teleop_max_linear_x_mps`: default `1.00`
- `teleop_max_angular_z_radps`: default `0.55`
- `teleop_allow_reverse`: default `true` for mapping teleop only
- `teleop_require_mapping_active`: default `true`
- `teleop_watchdog_timeout_sec`: default `0.5`

During a WebSocket teleop session, the server publishes `/ranger_mini3/teleop_allow_reverse=true` so `robot_safety` accepts low-speed reverse commands. When the WebSocket disconnects, times out, or receives `stop`, the server publishes one zero `Twist` and disables teleop reverse again. When no teleop session is active, it does not publish reverse-disable messages, so docking undock permissions remain independent.

The implementation boundary follows that lifecycle. `features/teleop/teleop_module`
owns WebSocket admission and the frame loop, ROS command/reverse publishers,
the repeat timer, session state, subscription lease refresh, mapping/charging/
elevator checks, state JSON, and disconnect cleanup. `TeleopFeatureModule`
constructs it and provides token, mapping/pose snapshot, BMS contact, elevator
admission, subscription, and process-running ports; the application composition
module owns only that aggregate. This extraction
preserves all existing checks and stop ordering; it adds no gate and changes no
teleop limit, timeout, topic, or final velocity-chain ownership.

`features/teleop/teleop_configuration_module` owns all 11 Teleop ROS
parameters and preserves their deployed defaults and bounds. Its only
cross-module input is the subscription module's normalized maximum lease TTL;
subscription parameters remain owned and declared by `application/subscriptions`.

## 2D Map PNG

`POST /api/v1/mapping/2d/start` starts the same repository-owned `slam_toolbox` chain used by the debug Web 2D mapping path. If navigation is active, the asynchronous transition cancels the active Nav2 task, publishes zero velocity, stops only navigation/localization mode services while keeping common services alive, clears the transient map context, and then launches mapping.

`POST /api/v1/mapping/2d/stop` stops the 2D mapping-side chain and returns `{"ok":true,"mapping_active":false}`. The stop scope includes `run_projected_map.sh`, `slam_toolbox`, scan preprocessing / republishing nodes, the C++ mapping-only odom bridge, and the mapping-owned FAST-LIO2 process marked with `NJRH_SLAM2D_PRIVATE_FASTLIO=1`. It does not kill chassis, canonical TF, local state, safety, or the API server.

The implementation boundary mirrors this lifecycle. `features/mapping/mapping_module` owns the complete API-facing vertical slice: HTTP dispatch, ROS `/map` cache, exact `/scan` ownership proof, asynchronous transition order, process and start-job collaboration, save/stop behavior, and status snapshots. `features/mapping/runtime/mapping_process_runtime` owns the launcher PID/process group, mapping-marked residual sweep, graceful signal escalation, and temporary LiDAR RPS/XPS restoration. `features/mapping/runtime/mapping_start_job` owns the thread-safe App-visible start transaction. `MappingFeatureModule` supplies the explicit floor/elevator/navigation/runtime-context ports; subscription, teleop, elevator and status aggregates consume its narrow interface directly. The application composition module only owns and connects the aggregate. This extraction adds no gate and changes no mapping, TF, DDS, pointcloud, LaserScan, or velocity-chain parameter.

`POST /api/v1/mapping/2d/save` accepts `building_id`, `floor_id`, and a business `map_name`. The server generates a stable `map_id`, preserves the original name as `display_name`, writes the map bundle under `maps_release/<building_id>/<floor_id>/maps/<map_id>/`, and then runs the same mapping-chain stop logic as `/mapping/2d/stop`. It intentionally does not activate the saved map. Runtime consumers keep reading fixed role files from `current/nav/nav_map.yaml` and `current/localizer/localizer_map.png`; those files change only after a later explicit `/api/v1/floors/switch` map selection.

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_name": "一楼大厅配送图"
}
```

Within the map bundle, nav files use a safe filename stem derived from
`map_name`, for example `nav/<safe_map_name>.yaml/.pgm`; Isaac localization
uses `localizer/<safe_map_name>.png`. `manifest.json` also records the strict
v2 epoch/digest identity in addition to map metadata and asset paths.

`GET /api/v1/maps` returns `floor_maps[]` for real map records and keeps `floors[]` only as a compatibility floor/current-map view. `floors[]` should not be treated as the map list. Each map entry includes `map_info` parsed from the Nav2 map YAML and image header so App-side taps can be converted to real map coordinates without assuming PNG scale:

```json
{
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "Lobby delivery map",
  "asset_epoch": 17,
  "asset_digest": "sha256:<64 lowercase hex>",
  "map_info": {
    "width": 242,
    "height": 103,
    "resolution": 0.05,
    "origin": [-3.2, -1.8, 0.0]
  }
}
```

`POST /api/v1/floors/switch` accepts `map_id` or `map_name`; `map_id` is
preferred. In selection-only mode it first waits for the legacy floor service
to validate the request, and only after a successful response activates the
selected manifest into `current/`. A failed service request therefore does not
change the active manifest.

The confirmed-runtime same-map response described above is the only exception:
it performs no selection at all and never contacts the service. It is not a
live floor switch.

Activation treats `maps_release/<building_id>/<floor_id>/current/` as a backend-owned runtime mirror. If an older dashboard or root-run process left `current/` as a non-empty real directory, the API first attempts normal removal, then safely quarantines the stale directory inside the same floor folder before creating a fresh runtime mirror. App clients should not write into `current/` directly.

`POST /api/v1/maps/delete` deletes only by `map_id`:

```json
{"map_id":"map_20260520T120000Z_012345abcd"}
```

The endpoint refuses building/floor-only deletion so a phone client cannot
accidentally remove an entire building or floor asset tree. It also serializes
against elevator draft/publish/rollback. A map referenced by the authoritative
current elevator release returns HTTP `409 ELEVATOR_CONFIG_MAP_IN_USE`; edit,
validate, and publish a release that no longer references that map before
deleting it.

## Semantic Poses And Navigation Goals

Semantic App points and keepout edits are stored as backend-owned overlay assets, not baked into `nav_map.pgm` or `localizer_map.png`. Android should not restore these overlays from local files.

Read the complete editable overlay layer:

```http
GET /api/v1/maps/semantic_layer?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

The response includes `poses[]`, `filters.keepout`, `keepout_mask.yaml/.pgm` paths, and any App-authored `filters/keepout_semantic_layer.json` content. If a client needs only keepout:

```http
GET /api/v1/maps/filters/keepout?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

Replace the complete keepout layer:

```http
POST /api/v1/maps/filters/keepout/save
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "keepout_lines": [
    {
      "id": "door_barrier_1",
      "name": "door barrier",
      "width_m": 0.6,
      "points": [{"x": 1.2, "y": 3.4}, {"x": 2.8, "y": 3.4}]
    }
  ],
  "keepout_polygons": [],
  "expected_revision": "keepout-v1-fnv64-0123456789abcdef",
  "reload_filter": true
}
```

Coordinates are finite map-frame metres. `width_m` is the full physical line width. The renderer applies the Nav map's complete `[origin_x, origin_y, origin_yaw]`, converts ROS bottom-left grid rows to PGM top-left rows, and conservatively marks cells touched by lines or polygons. Sending both arrays empty clears the complete layer and restores an all-neutral mask.

The backend canonicalizes and writes `keepout_semantic_layer.json`, `keepout_mask.yaml`, `keepout_mask.pgm`, and `keepout_commit.json` for the immutable map bundle, fixed active projections, and the actual Nav2 runtime staging directory. Each feature must hit the map and at least one free nav cell. It rejects malformed/self-intersecting geometry, duplicate IDs, excessive raster work, a layer that blocks every map cell, an older non-neutral mask with no editable semantic source, and semantic/PGM disagreement. File replacement uses checked write/fsync/rename, parent-directory fsync, whole-transaction readback, and rollback. The commit marker is written last so startup can fail closed after an interrupted multi-file update.

`GET /api/v1/maps/filters/keepout` returns `revision` / `keepout_revision`. Clients should send that value as `expected_revision` on full-layer replacement. A stale value returns HTTP `412 REVISION_CONFLICT` without writing. The revision is computed from canonical geometry, not JSON whitespace, `reload_filter`, or the expected revision itself.

When the exact map is loaded by an idle navigation runtime, `reload_filter` must remain `true`. Success then requires all of these proofs: the mask and filter-info lifecycle servers and global costmap are active, `KeepoutFilter` is enabled, `LoadMap` succeeds, a fresh `/keepout_filter_mask` message matches the candidate geometry and occupancy digest, the global costmap clear service acknowledges, and a full global costmap with a header stamp strictly after that clear is published. Per-feature samples added by the update must be exactly lethal (`100`, not inflation value `99`); sampled cells removed by the update must no longer be lethal or unknown. The global-costmap subscription exists only for this bounded proof and is destroyed before the request returns. The response uses:

- `outcome: "APPLIED"`, `runtime_selected: true`, and `runtime_effective: true` for a proven live update.
- `outcome: "SAVED_DEFERRED_INACTIVE"`, `runtime_selected: false`, and `effective_on_next_activation: true` when the target map is not currently loaded.
- `outcome: "NO_CHANGE"` for an identical inactive replacement.

A changed, unbound keepout layer recomputes the complete map bundle digest and
atomically stamps a new monotonic `asset_epoch`; `NO_CHANGE` does not consume
an epoch. If that identity stamp cannot be completed after the keepout
transaction, integrity is latched degraded and later admission fails closed.
The latch is persisted before the first payload write. Restart recovery keeps
it repairable only for the same map when the descriptor-pinned digest of all
nine non-keepout roles still matches; any extra map fault, missing map, or
non-keepout drift converts the global latch to an unrepairable state.

Map activation separately persists a transaction journal before changing
active manifests or the `current/` and floor compatibility views. Projection
files and their child/root/parent directories are fsynced before that journal
is removed, and startup replays a surviving journal. Live floor switching
nonetheless remains disabled: compatibility projection still reopens some
source paths and is not yet end-to-end descriptor-pinned from the verified
snapshot through publication.

The endpoint shares an admission barrier with navigation, mapping, docking, floor switching, map deletion, and map saving. It also checks the Nav2 action-status topic and requires stable stopped wheel odometry before an active-map update. Rollback ambiguity latches `keepout_integrity_degraded`, reports persistence/effectiveness as unknown, and blocks motion admission until a successful replacement re-proves integrity. The endpoint never cancels Nav2, sends velocity, or restarts a node. Neutral keepout mask servers/plugins remain resident by default so the first line can be applied without a Nav2 restart.

Read points for an existing map:

```http
GET /api/v1/maps/poses?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

`map_id` is preferred and reads that exact `maps/<map_id>/poses.yaml`. If `map_id` is omitted, the server reads the active map for the requested `building_id` and `floor_id`; `map_name` / `display_name` is accepted only as a fallback selector.

```json
{
  "ok": true,
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "display_name": "test-17",
  "poses": [
    {"pose_id": "delivery_123456", "type": "delivery_point", "name": "Point A", "x": 5.75, "y": -0.82, "yaw": 1.57}
  ]
}
```

Create or upsert a point:

```http
POST /api/v1/maps/poses
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","pose_id":"delivery_123456","type":"delivery_point","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}
```

Update a point by stable ID:

```http
PUT /api/v1/maps/poses/delivery_123456
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}
```

Delete a point by stable ID:

```http
DELETE /api/v1/maps/poses/delivery_123456?building_id=B1&floor_id=F1&map_id=map_20260520T120000Z_012345abcd
```

Replace the full point set for admin tools or batch import:

```http
PUT /api/v1/maps/poses/batch
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","poses":[{"pose_id":"delivery_123456","type":"delivery_point","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}]}
```

Batch replacement writes exactly the submitted list; `"poses":[]` clears all semantic points for that selected map. Active map changes are synchronized to `current/poses.yaml` so task navigation reads the same backend state the App just edited.

Legacy save-or-update endpoint kept for existing clients:

```http
POST /api/v1/maps/poses/save
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","pose_id":"delivery_123456","type":"delivery_point","name":"Point A","x":5.75,"y":-0.82,"yaw":1.57}
```

If `map_id` is omitted, the active floor map receives the pose. Active map edits are synchronized back into `current/`.

Preview the current robot pose in map coordinates:

```http
GET /api/v1/robot/pose
```

```json
{
  "ok": true,
  "frame_id": "map",
  "child_frame_id": "base_link",
  "x": 1.23,
  "y": -0.45,
  "yaw": 0.78,
  "stamp": 1770000000.123,
  "age_sec": 0.03,
  "map_id": "map_20260520T120000Z_012345abcd",
  "floor_id": "F1",
  "building_id": "B1"
}
```

If the server cannot read a fresh `map -> base_link` pose within `robot_pose_freshness_sec` (default `0.5` seconds), it returns `503` and never returns an old cached pose:

```json
{"ok":false,"error":"no fresh map-frame robot pose","frame_id":"map","child_frame_id":"base_link","age_sec":null}
```

The map identity attached to this pose is not discovered by scanning every active floor map. Navigation writes a runtime map context when a floor switch starts and marks it confirmed only after localization has produced `map->odom` and Nav2 has a ready global costmap. While that context is missing or still pending, `/api/v1/robot/pose` returns `503` during navigation/docking instead of attaching a stale map such as a previous floor's `building_id` / `floor_id`.

The App may keep navigation/localization resident while collecting or updating
points on this exact runtime map. The editor must compare the returned
`building_id/floor_id/map_id` with its selected map and wait for the robot to
settle before capture. Editor-map selection itself is App-local and must not
call `/api/v1/floors/switch`.

Save the robot's current map-frame pose as a semantic point:

```http
POST /api/v1/maps/poses/save_current
```

```json
{"building_id":"B1","floor_id":"F1","map_id":"map_20260520T120000Z_012345abcd","type":"delivery_point","name":"Current Point"}
```

The server requires the same fresh `map -> base_link` pose as `GET /api/v1/robot/pose`, writes the selected `maps/<map_id>/poses.yaml`, and synchronizes `current/poses.yaml` when that map is active. If no `pose_id` is supplied, the server generates one. Request-body `yaw` / `theta` is intentionally ignored; saved `x/y/yaw` must equal the live robot pose. For `type: "dock"`, the saved pose is the final charging-contact `base_link` pose, not the pre-dock approach point. This endpoint is for live field marking; static map editing should keep using `POST /api/v1/maps/poses`.

Elevator-internal roles are deliberately excluded. For schema-v3
`hall_call/landing/cabin/cabin_panel`, read `/api/v1/robot/pose`, verify its exact map
identity, copy `x/y/yaw` into the in-memory elevator configuration, and save the
complete document through `PUT /api/v1/elevator-config/draft`.

Send the robot to a saved point through Nav2:

```http
POST /api/v1/navigation/goal
```

```json
{"building_id":"B1","floor_id":"F1","pose_id":"delivery_123456"}
```

The API server reads `maps_release/<building_id>/<floor_id>/poses.yaml` and queues a background `NavigateToPose` send to `/navigate_to_pose`. Direct map-frame goals are also accepted with `x`, `y`, and `yaw`, but phone clients should normally use `pose_id` so the car remains the source of truth for floor assets. Goal and cancel calls are serialized around the Nav2 action client; transient rclcpp action-client exceptions are captured by the background job or logged without taking down port 8080. All HTTP handlers are wrapped by a request-level exception guard, so individual ROS service/action/file failures return JSON errors instead of aborting the API process. Successful acceptance returns `navigation_goal_id`; Nav2 action-send failure and completion are reported by the background `navigation_goal` object in `/api/v1/status` and `/api/v1/navigation/state`.

Delivery completion is policy-driven and has one safety-owned command chain. Normal delivery defaults to `goal_completion_policy=pose_required`, so target x/y/yaw is sent in the Nav2 action and Nav2 native `RotationShimController + SimpleGoalChecker(stateful=false)` handles the primary XY+yaw approach. The default Nav2 pose gate is 0.06 m XY and 0.05 rad yaw. After any Nav2 result, the API waits for bridge smoothing, re-reads a fresh `map -> base_link`, and only sets `task_complete=true` when the commercial gate is satisfied. Active bridge smoothing/correction still blocks final completion, but clean gated-AMCL stationary standby (`amcl_static_standby=true` and `amcl_not_moving_no_update_ok=true`) is tolerated even if the AMCL status file still reports `amcl_correction_pending=true`. A 0.06-0.12 m XY overrun or 0.05-0.15 rad yaw overrun triggers bounded same-goal retry or final yaw alignment; API final yaw targets 0.045 rad internally before the unchanged 0.05 rad commercial gate. A 0.12-0.35 m XY overrun or 0.15-0.35 rad yaw overrun enters recovery retry. During those final-verify retries the API publishes a short-lived `/ranger_mini3/allow_reverse` permit so `robot_safety` can allow bounded overshoot correction; outside that permit, the safety layer clamps ordinary reverse. If retries cannot satisfy the gate, the job is marked `degraded` with `task_complete=false` instead of being reported as arrived. Post-retry XY within 0.08 m may be accepted through `post_nav2_final_verify_acceptance_slack_m=0.02`. `position_only` remains an explicit engineering opt-out when final heading is irrelevant.

API-owned ordinary final yaw still checks actual chassis stop before declaring success. The legacy predock yaw servo uses the same rule only when `docking_delegate_staging_motion_to_manager=false` is explicitly selected for rollback. In either API-owned path, non-zero yaw commands reset the stop-stability counter; the server sends zero before the target, waits for stable `/wheel/odom.twist.twist.angular.z`, then re-reads the pose. Production predock physical alignment instead belongs to `robot_docking_manager`.

The production API does not proactively cancel an executing near-goal Nav2 action: `navigation_near_goal_stalled_handoff_enabled=false`. `GoalScopedRotationShimController` owns the bounded non-Ackermann terminal residual while the same `FollowPath` action remains active, so controller success and progress-checker state stay coherent. The API deterministic terminal servo remains the fallback only after a true Nav2 abort. That fallback decomposes the target error into signed yaw, body-frame forward error, and body-frame lateral error, then corrects yaw first with pure `angular.z`, lateral second with pure `linear.y` in `side_slip`, and forward/reverse third with pure `linear.x`. Neither owner loosens the 0.06 m / 0.05 rad acceptance gate, and Ranger Mini3 mixed x/y commands remain prohibited.

The `navigation_goal` JSON exposes final verification and recovery diagnostics for the App and field logs: `final_pose_verified`, `task_complete`, `final_pose_verify_reason`, `final_verify_retry_count`, `final_verify_retry_reason`, `final_verify_retry_goal_sent`, `final_verify_xy_error_m`, `final_verify_yaw_error_rad`, `final_verify_failure_is_terminal`, `final_yaw_align_attempted`, `final_yaw_align_blocked_reason`, `final_yaw_align_duration_sec`, `final_yaw_align_timeout_sec`, `final_yaw_align_target_yaw_rad`, `final_yaw_align_initial_yaw_error_rad`, `final_yaw_align_final_yaw_error_rad`, `final_yaw_align_max_xy_drift_m`, `final_yaw_align_observed_xy_drift_m`, `final_yaw_align_cmd_topic`, and `final_yaw_align_bypass_collision_monitor`. App clients should show ordinary point success only when `task_complete=true`.

When the backend state is `docked`, `/docking/status` starts with `docked` or `charging`, stable BMS charging contact is fresh, or valid non-stale latch evidence is present, `/api/v1/navigation/goal` accepts the navigation job and the background job automatically performs controlled undocking before sending the Nav2 goal. The same snapshot is available through read-only `GET /api/v1/navigation/pre_goal_check` and through `/api/v1/status` / `/api/v1/docking/state` as `pre_navigation_dock_check`. The read-only endpoint keeps the existing `would_auto_undock` field and also exposes `auto_undock_required`. The snapshot exposes `api_bms_charging_contact`, `api_bms_charging_contact_stable`, `api_bms_charging_contact_reason`, `dock_contact_snapshot`, `dock_contact_latch_age_sec`, `dock_contact_latch_stale`, `dock_contact_latch_contradicted_by_live_state`, `dock_contact_latch_auto_cleared`, `strong_live_docked`, `latch_valid_for_auto_undock`, `docked_state_class`, `docked_evidence`, `docked_warnings`, `bms.power_supply_status`, `bms.current`, `docking.last_status`, `final_is_docked_or_charging`, `final_auto_undock_required`, and `auto_undock_reason`, so a full battery with `current=0` is still diagnosable through `POWER_SUPPLY_STATUS_FULL` or the BMS contact reason. If BMS reports no contact because the charger signal is missing or the robot was manually pushed onto the dock, maintenance can call `POST /api/v1/docking/confirm_docked`; this writes only the latch and sends no velocity. `POST /api/v1/docking/clear_docked_latch` clears only that latch and also sends no velocity. `scripts/jetson/runtime_overlay/scripts/verify_dock_contact_latch_gate.sh` checks the latch file, API gate, `/docking/status`, and `/battery_state` without moving the robot; `--clear-stale-bms-latch` is the explicit operator path to clear old BMS latch evidence. Because `robot_docking_manager` is normally resident, `/docking/undock` should already be available; the API start command remains only as a fallback when the resident service is absent. After `/docking/status` reports odometry-confirmed `undocked`, the background job calls `/global_localization/trigger`; that wrapper is the sole bridge-arm/Isaac-dispatch owner. The job waits until the fresh accepted result is reflected in `map -> base_link`, and then releases the held Nav2 send. If undocking, post-undock relocalization, or bridge acceptance fails or times out, the accepted navigation job moves to failed state and no Nav2 goal is sent. Goal responses include `pre_navigation_undock`, `pre_navigation_undock_detail`, and `pre_navigation_dock_check` so the App can show whether departure from the charger is queued.

`/api/v1/status` and `/api/v1/navigation/state` also expose `safety.status`, `safety.motion_allowed`, `navigation.blocked_by_docked_contact`, and `navigation.normal_motion_blocked_reason`. If the API gate is bypassed and `robot_safety` blocks a normal command while dock/contact evidence is active, the App can display `DOCKED_CONTACT_BLOCK` without inferring dock state from position.

Field diagnostic:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_pre_navigation_undock_gate.sh \
  --building-id building_1 --floor-id floor_1 --pose-id delivery_1

bash scripts/jetson/runtime_overlay/scripts/verify_docked_navigation_undock_gate.sh \
  --building-id building_1 --floor-id floor_1 --pose-id delivery_1
```

The scripts are read-only unless `--execute-goal` is supplied. The docked interlock verifier also has optional `--test-normal-cmd-block` and `--test-docking-cmd-allowed` checks that publish one low test command; use them only during controlled bench validation.

Cancel requests are wired to Nav2, not only acknowledged over HTTP:

```http
POST /api/v1/navigation/cancel
```

By default this is a task cancel, not a process stop. The HTTP handler publishes a zero burst to `/cmd_vel_api`, creates a background cancel job, and returns `202 Accepted` quickly. The background job uses `navigation_cancel_action_wait_sec` as a short `/navigate_to_pose` action-server probe, cancels the cached API-started `NavigateToPose` goal handle when the action server is available, sends cancel-all to `/navigate_to_pose`, publishes another zero burst, and leaves the resident Nav2/localization runtime alive.

All API-owned Twist command publishers use `KEEP_LAST(1)` QoS. `/cmd_vel_api` is a latest-command control stream, not a queued instruction log; a zero command must replace older nonzero yaw/linear commands immediately before `robot_safety` arbitrates the final `/cmd_vel`.

The endpoint remains idempotent for task exit: if the Nav2 action server is already gone, it still publishes zero velocity and records the cancel job failure reason without tearing down localization.

For diagnostics or recovery, call `POST /api/v1/navigation/stop` or `POST /api/v1/navigation/stop_runtime`. These endpoints force the same background cancel job with `stop_stack=true`: they publish zero velocity, request cancellation, and terminate the resident Nav2/localization runtime only after Nav2 terminal evidence is proven. An accepted cancel-all response with an empty `goals_canceling` list is immediate proof that no cancelable goal remained; it must not wait for a status frame that an idle action server may never publish. If cancellation is rejected, times out, or otherwise remains unproven, the stop script is not invoked, the cancel job fails, and Nav2 remains alive so late terminal evidence can still be observed. The stop script tears down Nav2/localization process patterns before bounded AMCL cleanup, so AMCL lifecycle waits cannot consume the API stop window before Nav2 has been cleaned up. Startup failure before confirmed `ready` also rolls back both Nav2 and the occupancy-localization helper layer so the next resume starts from a clean process set. The App's return-to-charger action should not call these endpoints first; it should call `/api/v1/docking/start` directly so the backend can switch modes without stopping Nav2 and localization.

`navigation_cancel_http_smoke` covers the idle-runtime regression where the old implementation reported `cancel_all_ok=false` but still stopped Nav2 and returned success, leaving `delayed_side_effect_unknown_count=1`. The required postcondition is a successful, proven stop with that counter still at zero.

Poll the latest cancel job and goal job with:

```http
GET /api/v1/navigation/state
```

`GET /api/v1/mapping/2d/map` returns `Content-Type: image/png` for the live `slam_toolbox` `/map` cached after that App-started mapping session begins. It does not return a map_server static map or historical saved map by default.

## Docking

Docking is a two-stage backend workflow. The App stores a dock pose in the same `poses.yaml` mechanism as delivery points, normally with `type: "dock"`. That pose represents the final robot `base_link` pose when the front charging contacts are aligned with the physical dock. The API computes the pre-dock pose by backing up from that final pose along its yaw by `docking_pre_dock_distance_m` (default `0.60 m`).

Start docking:

```http
POST /api/v1/docking/start
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_20260520T120000Z_012345abcd",
  "dock_id": "dock_main",
  "resume_navigation": true
}
```

The docking endpoint never activates a map. `map_id` must already be the active
manifest and must exactly match the confirmed, ready runtime context
(`building_id/floor_id/map_id`); otherwise admission fails with
`FLOOR_SWITCH_REQUIRED`. The backend rechecks that identity immediately before
accepting the job, then starts/reuses only that same-map navigation runtime,
cancels any cached API navigation goal without stopping the Nav2/localization
stack, checks bridge `safe_for_goal_start`, and sends Nav2 toward the pre-dock
pose as coarse approach. The remaining manual pre-dock, staging, bridge-settle,
and fine-docking gates are unchanged.
`docking_manual_predock_distance_check_enable` is `false` by default, so the
manual pre-dock distance check remains optional while yaw sanity still applies.
The complete commissioning lookup and validation policy lives in
`features/docking/configuration/docking_predock_pose_resolver`: an explicit
`predock_pose_id` wins, followed by the existing dock-specific ID/name
conventions and finally a unique predock-type fallback. Ambiguous matches retain
their `409` response, missing explicit IDs retain `404`, and unsafe IDs retain
`400`. The composition root only injects read-only pose access and does not
reimplement those decisions.

Cancel docking:

```http
POST /api/v1/docking/cancel
```

This cancels the cached pre-dock Nav2 goal, calls `/docking/stop`, and publishes a zero velocity burst through the existing safe command path. The App should poll:

```http
GET /api/v1/docking/state
GET /api/v1/status
```

Undock from a charger:

```http
POST /api/v1/docking/undock
```

```json
{
  "dock_id": "dock_main",
  "reason": "app_manual_undock"
}
```

The undock endpoint is accepted only when the backend state is already docked, the API sees live charging contact, or the persistent dock-contact latch is explicitly set. If a non-undock docking job is still `running`, the undock request first atomically marks that owner canceled, publishes zero, cancels its Nav2 goal when present, stops the near-field manager, joins the old worker, releases its stale correction pause, and terminalizes the old job. Only after that proof does it create the controlled-undock job; this prevents a failed `FINE_ALIGN` owner from returning a permanent 409 while still forbidding two motion owners. If cleanup proves that no dock evidence remains, the endpoint returns `already_undocked=true` without sending velocity. It calls the resident `robot_docking_manager` through `/docking/undock`, starting it only as a fallback if the service is unexpectedly absent, and tracks the job as `undocking` until `/docking/status` reports `undocked` or `undock_failed...`. `robot_docking_manager` reads the same latch before accepting `/docking/undock`, so `pre_navigation_dock_check` must not claim `can_auto_undock=true` unless the controlled undock service will accept that dock evidence. Once odometry confirms departure, the API enters `relocalize_after_undock`, calls `/global_localization/trigger` without pre-arming the bridge, verifies that the resulting localization is reflected in `map -> base_link`, and records `post_undock_relocalization_*` fields in the docking job. Manual undock remains `undocked` even if the relocalization warning needs operator attention. Auto-undock before `/api/v1/navigation/goal` also remains `undocked` after physical departure; if post-undock relocalization or hard TF/bridge readiness fails, the pending Nav2 goal is not sent and `post_undock_navigation_readiness_failed=true` explains the blocker. Local costmap and AMCL scan-admission transients after an accepted post-undock relocalization are warnings, not blockers. Reverse motion remains inside `robot_docking_manager -> /cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base`; `/cmd_vel_safe` is diagnostic only. The configured undock speed is `0.50 m/s`, independently capped by `undock.max_speed_mps=0.50`; first-motion delay is handled by `undock.motion_start_timeout_s`, while `undock.no_progress_timeout_s` is reserved for a stall after movement has started.

`accepted=true` is an API admission result, not a synonym for the underlying ROS Trigger result. The undock response and `GET /api/v1/docking/state` expose `api_accepted`, `already_running`, `docking_service_called`, `docking_service_success`, `docking_service_message`, `docking_status_at_request`, `docking_status_after_request`, `undock_started_observed`, `undock_cmd_count_observed`, `undock_failure_reason`, and `docking_service_warning` so field diagnostics can distinguish API admission, `/docking/undock` service success, `/docking/status` observation, and downstream motion execution.

Relevant states are `accepted`, `relocalize_before_predock`, `nav_to_predock`, `STAGING_NAV2_EARLY_HANDOFF`, `STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK`, `relocalize_after_predock`, `fine_bridge_settle`, `fine_docking`, `relocalize_after_fine_docking`, `docked`, `undocking`, `relocalize_after_undock`, `undocked`, `failed`, and `canceled`.

Saved-map preview is explicit:

```text
GET /api/v1/mapping/2d/map?source=saved&map_id=map_...&building_id=B1&floor_id=F1
```

This is the production App lookup. It resolves the catalog manifest, verifies
that the optional building/floor selectors own the map ID, and reads only:

```text
maps_release/B1/F1/maps/map_.../localizer/<safe_map_name>.png
```

An exact request returns `404` when the ID or immutable PNG is missing and
`400` when the ID belongs to a different building or floor. These errors never
fall back to a newest PNG.

Legacy commissioning callers may still use
`?name=test-16` to read
`/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps/test-16.png`,
or selector-free `?source=saved` to request the legacy newest-preview behavior.
Saved mode reads an existing PNG asset; it does not convert PGM at request time.

If `api_token` is non-empty, requests must include:

```text
X-Robot-Token: <token>
```

## Elevator lock recovery wire contract (2026-08-05)

Every `/api/v1/elevator-test/state` response now projects
`physical_zone`, `interrupted_state`,
`interrupted_expected_confirmation`, and the server-authoritative
`allowed_recovery_actions` array. The commissioning App must render these
fields and must not reconstruct unlock permission from failure codes.

The field recovery request adds `action`, `physical_zone`,
`stationary_confirmed`, and `door_zone_clear_confirmed`. For
`CONFIRM_SOURCE_OUTSIDE_AND_RELEASE`, all three observations must be explicit,
the confirmed floor must equal the retained source/current floor, and the
server must have advertised that exact action for the current transaction
sequence. A previously journaled outside observation is retried only with
`RETRY_SAFETY_VERIFICATION`. A `409` always requires a fresh state query and
never authorizes replaying the old sequence.

## Ordinary navigation execution ownership

The complete concrete goal-execution edge now lives in
`features/navigation/mission/navigation_goal_execution_module`. After
`NavigationModule` admits a goal, it preserves the existing pre-navigation
undock/readiness sequence, Nav2 action-send evidence, terminal-result wait,
near-goal handoff, bridge smoothing, commercial final-pose verification,
same-goal retry, lateral/longitudinal correction, and final-yaw alignment.
The same module owns the `/cmd_vel_docking` predock-versus-ordinary-yaw mutex;
the composition root supplies narrow floor, localization, safety, docking and
runtime-state ports. `DockingJobExecutor` receives the dedicated
`DockingJobExecutionModule`, so the composition root no longer implements its
port or retains its forwarding methods.

This is an ownership-only extraction. Goal tolerances, retry counts, timeout
budgets, phase/error text, AMCL no-motion policy, correction-pause semantics,
speed limits, reverse permits, and the final Nav2-to-safety velocity chain are
unchanged. The module does not publish TF and does not create a path around
`robot_safety`.
