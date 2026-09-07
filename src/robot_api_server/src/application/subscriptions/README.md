# Application subscriptions

This directory owns the complete App page-subscription module:

- `subscription_module` is the deep public boundary. It dispatches
  `POST /api/v1/subscriptions/acquire|heartbeat|release`, preserves legacy
  client-ID aliases and the compatibility lease, owns the one-second expiry
  timer, maps resource transitions to narrow cross-domain ports, and owns the
  page-scoped `/scan` subscription/cache lifecycle.
- `subscription_manager` is the thread-safe lease/refcount engine. It invokes
  transitions outside its mutex when a resource changes between zero and one
  clients.
- `subscription_api` contains the established request parsing, ID validation,
  resource sorting/deduplication, TTL clamping, and response-list helpers.
- `subscription_configuration_module` uniquely declares the scan topic,
  scan-freshness threshold, and default/maximum lease TTL parameters. The
  runtime module retains the established minimum/default/maximum
  normalization before serving leases.

The public seam is intentionally small: `handle_http`, teleop's internal
`acquire`/`release`, `snapshot_json`, and read-only scan diagnostics. The
composition root supplies only four cross-domain ports for resident status,
page-owned live map, resident TF, and final teleop command clearing.

Resource semantics remain unchanged: `status` and `tf` are process-resident;
`live_map` follows page refcounts; high-rate `/scan` is created on `0 -> 1` and
released with its cache cleared on `1 -> 0`; final `teleop` release clears the
cached command. No DDS, TF, scan geometry/QoS/timestamp, mapping, navigation,
safety, elevator, docking, or velocity parameter is changed by this ownership
move.
