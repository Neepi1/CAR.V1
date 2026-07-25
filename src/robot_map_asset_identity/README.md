# robot_map_asset_identity

Pure C++ ownership boundary for immutable map-asset identity:

- SHA-256 and the `njrh-map-asset-bundle-v1` canonical digest wire format.
- A chunked `CanonicalMapAssetDigestStream` that produces the identical wire
  digest without retaining whole map images in memory.
- A persistent, process-safe, globally monotonic `uint64` `asset_epoch`.

`PersistentAssetEpochRegistry` stores state below
`<maps_root>/.map_asset_registry`. Binding the same
`building_id + floor_id + map_id + digest` is idempotent. A new key or changed
digest consumes a new epoch. Changing a key back to a previously used digest
also consumes a new epoch; historical epochs are never reused. Epoch gaps are
allowed, but epoch rollback is not.

The registry writes the high watermark before replacing bindings. Both files
use temporary-file + fsync + rename + parent-directory fsync on Linux, and all
operations are serialized with `flock`. Missing high-watermark state is
recovered from intact bindings; present but malformed or regressed state fails
closed. Before every bind, all regular `manifest.json` files below `maps_root`
are parsed as strict JSON and the maximum committed v2 `asset_epoch` is
audited. Any traversal/read/parse uncertainty, or a registry high watermark
below that maximum (including restoration of an older non-empty registry),
blocks allocation instead of silently raising the watermark. Registry recovery
must preserve or reconstruct the old high watermark first.

The persisted binding set is capped at 100,000 records and 32 MiB. Both limits
are preflighted before the high watermark is advanced, so a successful write
cannot create a registry that the next process refuses to read. `lookup()` is
strictly read-only: it takes a shared read-only lock, never creates a registry
or lock file, and fails closed if either coordination path is absent.
