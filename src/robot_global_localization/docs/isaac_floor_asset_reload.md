# Isaac floor asset replacement

## Ownership

`/global_localization/apply_floor_assets` is a lower-level step of the atomic
floor transaction. `robot_floor_manager` must call it only after the
transaction-scoped motion hold and correction pause are proven. The service
does not grant motion, stop navigation, update `map -> odom`, or select the
floor context by itself.

Only the Isaac composable node is replaced. The component container and the
global-localization wrapper normally remain resident. The production container
is Humble's `component_container_isolated`: each component has a dedicated
single-threaded executor that is canceled and joined before destruction. The
launch owner has bounded process respawn solely to recreate an empty component
manager after an abnormal exit; it does not reload a component or authorize a
floor switch. This is not a single-node restart and must not be used as an
ad-hoc recovery action.

## Success proof

A successful response proves all of the following within the service call:

- the exact request identity and all paths passed strict admission;
- exactly one configured Isaac component was captured;
- every declared current parameter was read before unload;
- the target component was constructed with target map fields and preserved
  tuning;
- `load_node` returned success before the timeout;
- a fresh `list_nodes` response contained exactly the returned full name and
  unique ID.

The response echoes `transaction_id`, `building_id`, `floor_id`, `map_id`,
`asset_epoch`, and `asset_digest`. The caller must compare all fields. The
reliable transient-local `/global_localization/asset_state` message is the
second normal-path proof and the transport-outcome reconciliation source
described below.

`localizer_generation` counts confirmed component constructions after the
latest durable seed. A confirmed rollback therefore advances the generation
even though the requested target failed. Exact-identity idempotence neither
unloads nor advances the generation. A process restart may restore the prior
nonzero value only through the restart proof below.

If the Apply RPC response is delayed or lost, the floor manager does not repeat
the component replacement and does not fail at the generic 10 second service
timeout. Within its bounded 30 second localizer-apply budget it accepts only a
fresh terminal typed state that has the same transaction, exact requested and
active identity, `success=true`, `reloaded=true`, a generation newer than the
transaction baseline, and `localizer_ready=true`. An exact terminal failure is
reported as failure. Applying, stale, foreign, floor-only, or non-advancing
state is not success.

The wrapper creates the Isaac trigger service client once during node
construction. ROS 2 rediscovers the same-named trigger server after component
replacement. The wrapper therefore never removes or adds that client waitable
from a Reentrant callback while its multi-threaded executor is spinning.

## Restart identity bootstrap

The wrapper starts with no authoritative active identity. It reads the
parameterized `runtime_map_context_file` only after the ROS executor is
spinning, then retries at `runtime_context_bootstrap_retry_sec` without
unloading or loading the Isaac component.

`BOOTSTRAP_READY` is published only when one proof chain is complete:

1. the durable context is a bounded, regular, non-symlink document with schema
   `njrh.runtime_map_context.v1`, `state=ready`, `confirmed=true`, a safe
   transaction ID, exact safe building/floor/map IDs, positive `asset_epoch`,
   canonical lowercase SHA-256 digest, positive `localizer_generation`,
   positive `explicit_relocalization_sequence`, and a finite positive
   `updated_at`;
2. the corresponding fixed-role `current/` files pass the same path, file,
   PNG, and localizer-YAML admission used by a live apply;
3. the active `current/manifest.json` is schema
   `njrh.map_manifest.v2`, declares
   `sha256`/`njrh-map-asset-bundle-v1`, and its
   building/floor/map/epoch/digest exactly equal the durable context;
4. composition preflight succeeds and exactly one configured Isaac component
   is captured; and
5. its typed `map_yaml_path`, `image`, `resolution`, `origin`, and
   `occupied_thresh` equal the fixed-role current localizer YAML and PNG.

Only then does the wrapper initialize `active_identity_valid=true`,
`localizer_ready=true`, and seed the persisted nonzero generation. It publishes
the reliable transient-local `LocalizerAssetState` with code
`BOOTSTRAP_READY`. All missing legacy fields, duplicate fields, unresolved
paths, manifest identity drift, component ambiguity, missing/wrong parameter
types, and map-value drift fail closed with a `BOOTSTRAP_*` code, both active
flags false, and generation zero. A failed bootstrap is evidence that identity
is unknown; it is not a request to replace the component.

## Failure and rollback

Validation, missing services, ambiguous component identity, incomplete
parameter capture, and unload failure are all closed failures. A target
load failure remains a failed request even when rollback succeeds.

Rollback uses the complete parameter snapshot captured before unload. It is
attempted only when the adapter has confirmed that no matching target component
remains. If the component graph is unavailable or ambiguous after a timeout,
the wrapper does not risk loading a duplicate component; it reports
`localizer_ready=false` and leaves recovery to the still-held outer floor
transaction.

The ROS adapter waits through bounded, transient `list_nodes` ambiguity instead
of deciding from the first post-unload sample. Success requires the unload
response itself to be successful, exact old-component absence, and live
list/load/unload manager services. A missing unload response is still a failed
operation even if a respawned manager later proves the component absent; that
evidence permits only the existing captured-source rollback path. This preserves
the delayed-side-effect rule while avoiding a permanent recovery lock when the
source component can be restored. Persistent ambiguity remains fail-closed.

## Required field validation

- IDs are bounded safe identifiers.
- `asset_epoch` is positive.
- `asset_digest` is canonical lowercase `sha256:<64 hex>`.
- all asset paths are absolute regular files beneath `floor_asset_root`;
  parent traversal, intermediate symlinks, empty files, and oversized files
  are rejected;
- the PNG signature must be valid;
- localizer YAML must contain a positive finite `resolution`, three finite
  `origin` values, a valid `occupied_thresh`, and an image path resolving
  exactly to `localizer_map_png`;
- component package/plugin/node identity and remaps must reconstruct the exact
  configured full node name.

The floor manager remains responsible for verifying that the supplied asset
digest is the digest of the frozen published release before it invokes this
lower-level service.

## Hardware acceptance still required

With the robot stationary and the floor transaction hold active:

1. record component ID, parameters, typed asset state, and localization health;
2. apply one real target floor release;
3. verify one old-ID unload, one new-ID load, exact paths/identity, and one
   generation increment;
4. trigger localization and require bridge-approved fresh `map -> odom`;
5. inject a bad test-only target and verify rollback, failed outer transaction,
   and continuing motion hold;
6. confirm there is exactly one localizer component and no probe process left.

Also perform one cold wrapper restart after a completed floor transaction.
Record the durable context, current manifest, live component parameters, and
the latched typed state. Require one `BOOTSTRAP_READY` with the same exact
identity and persisted nonzero generation, with no composition unload/load.
Then repeat in an isolated test runtime with one mismatched live map parameter
and require a fail-closed `BOOTSTRAP_LIVE_LOCALIZER_PARAMETER_MISMATCH`.

No vehicle movement or live component replacement was performed as part of the
source-level implementation and isolated tests.
