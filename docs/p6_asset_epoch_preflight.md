# P6 Map Asset Epoch And Non-Moving Preflight

## Current boundary

P6 now has an authoritative map identity contract and a read-only source-bundle
verifier. This is non-moving preflight work only. It does not enable live
cross-floor switching.

`robot_floor_manager` still defaults `live_floor_switch_enabled` to `false`.
The read-only snapshot loader is not connected to the FloorSwitch Action, and
the current Action adapter cannot reload a localizer, mutate Nav2, send a goal,
or authorize motion. Keep that default until the real reload and readiness
evidence listed below has passed isolated integration and supervised hardware
acceptance.

## Exact identity

One map asset is identified by this indivisible tuple:

```text
(building_id, floor_id, map_id, asset_epoch, asset_digest)
```

`asset_epoch` is a positive `uint64`. `asset_digest` must be canonical lowercase
`sha256:<64 hex>` under the `njrh-map-asset-bundle-v1` contract. Matching only
the digest, only the epoch, or the three textual IDs is insufficient.

The map asset epoch is not an elevator release generation, draft revision,
localizer generation, or API job sequence. Those counters remain separate.

## Persistent global epoch registry

`robot_map_asset_identity::PersistentAssetEpochRegistry` owns the epoch state
under:

```text
<maps_root>/.map_asset_registry/
```

Epochs are globally monotonic across all building/floor/map keys in that
`maps_root`. Binding the same exact key and digest is idempotent. A new key, a
changed digest, or changing content back to an older digest consumes a new
epoch. Historical epochs are never reused; gaps are allowed.

Registry mutation is process-serialized and uses atomic durable replacement on
Linux. Corrupt or regressed state fails closed. If manifest-v2 assets remain
but the registry was lost, allocation also fails closed instead of restarting
at epoch 1. Every bind and lookup independently scans the exact committed
source-manifest layout and rejects a non-empty registry whose high watermark
is below the maximum committed manifest epoch. Unrelated JSON files and
elevator release manifests are not treated as source-map manifests. Registry
recovery must preserve or reconstruct the previous high watermark. Read-only
lookup creates no registry or lock files.

## Manifest v2 and canonical content

The authoritative source is:

```text
<maps_root>/<building_id>/<floor_id>/maps/<map_id>/
```

Its `manifest.json` uses schema `njrh.map_manifest.v2` and records:

```json
{
  "schema": "njrh.map_manifest.v2",
  "asset_epoch": 17,
  "asset_digest_algorithm": "sha256",
  "asset_digest_contract": "njrh-map-asset-bundle-v1",
  "asset_digest": "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
}
```

The canonical digest covers these 11 logical roles with their role names:

- Nav map YAML and PGM;
- localizer PNG and parameter YAML;
- keepout, speed, and binary mask YAML/PGM pairs;
- `reports/asset_report.json`.

`manifest.json` and `poses.yaml` must exist as authenticated regular files, but
they are deliberately excluded from that 11-role content digest. Manifest v2
is written with atomic replacement; partial identity fields, duplicate keys,
zero/non-integer epochs, unsupported algorithms/contracts, and non-canonical
digests are rejected.

## Read-only snapshot

`robot_floor_manager::FloorAssetSnapshotLoader` accepts the exact tuple plus
`maps_root`. It:

1. looks up the tuple in the authoritative registry;
2. opens only the matching source bundle;
3. validates manifest v2, exact role paths, layout, and required files;
4. recomputes the canonical 11-role digest;
5. rechecks file/directory identity and the registry before returning.

On Linux it rejects unsafe IDs, symlinked path components or assets, hard-linked
required assets, non-regular files, ambiguous map filenames, and files that
change during the read. It has no bind or write operation and no ROS,
map-server, localizer, bridge, costmap, goal, or velocity port.

## Non-moving preflight evidence

The current phase may be verified without moving the robot:

- registry tests prove global monotonic allocation, idempotence, concurrency,
  loss/corruption handling, and read-only lookup;
- manifest tests prove strict v2 parsing and atomic writing;
- snapshot tests prove exact tuple success and rejection of epoch, digest,
  content, layout, link, and race failures;
- elevator draft/release tests prove that the server stamps both identity
  fields and that publish/rollback reject drift;
- the synthetic cross-floor test composes the pure state-machine cores without
  ROS or physical motion;
- the exposed FloorSwitch Action remains preflight-only and ends blocked while
  live switching is disabled. Requested identity must not be reported as
  proven active identity.

Passing those checks proves contract integrity only. It does not prove that the
Isaac localizer loaded the requested PNG/parameters or that the runtime is safe
to resume.

## Crash recovery and integrity latch

Map activation writes a persistent
`.map_activation_transaction.v1` journal before changing active manifests or
compatibility projections. Source-manifest writes are atomic. Projection files
and their child/root/parent directories are fsynced before the journal is
durably removed, so a process restart can replay an incomplete activation.

Keepout updates persist `.map_asset_integrity_degraded.v1` before their first
payload write. Automatic repair is allowed only for the same `map_id` when the
descriptor-pinned digest of all nine non-keepout roles still matches the
stored proof. An unrelated map failure, a missing marked map, or any
non-keepout drift converts the latch to an unrepairable fail-closed state.
Startup preserves a repairable proof only when it uniquely explains a
keepout-only crash window.

## Gates before live floor switching

Do not connect or enable the live Action until all of the following exist and
are proven:

- a non-blocking transaction adapter consumes the exact read-only snapshot;
- compatibility projection is fully descriptor-pinned from the verified
  source snapshot through `current/` publication; current pathname-based
  reopen operations are not yet sufficient for adversarial live switching;
- the Isaac localizer performs a real target asset reload instead of caching
  paths, and exposes a new target-localization generation;
- `map -> odom` remains owned only by `robot_localization_bridge` and
  `odom -> base_link` only by `robot_local_state`;
- target building/floor/map/epoch/digest, localization health, bridge
  readiness, and fresh global/local costmaps agree before commit;
- motion hold, Nav2 idle, pause ownership, failure cleanup, and
  `robot_safety` admission are proven;
- isolated integration passes first, followed by one supervised runtime
  restart and controlled hardware acceptance.

Until then, do not use the current Action or offline floor selection as a way
to switch maps while the robot is in an elevator, and do not perform a real
cross-floor run.
