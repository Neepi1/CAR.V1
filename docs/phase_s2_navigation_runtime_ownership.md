# Phase S2 Navigation Runtime Ownership

## Problem

Historically, after boot the systemd resident navigation runtime could already
be starting or running while the App sent `POST /api/v1/floors/switch` with
`resume_navigation=true`. Starting a second runtime for the same
building/floor/map could make the new process clean up the existing
localization stack, which then caused the first resident runtime to stop Nav2.

The public endpoint now rejects `resume_navigation=true` with
`LIVE_FLOOR_SWITCH_DISABLED`. Ordinary App navigation startup instead uses the
separate `POST /api/v1/navigation/start` endpoint after offline exact-map
selection. The reuse logic below serves repository-controlled startup,
ordinary exact-map startup, and docking recovery. App live floor switching is
now a separate `/api/v1/floor-switch/start|state|cancel` transaction that
wraps the strict FloorSwitch Action and never forks a second runtime.

## Contract

- Only one resident navigation runtime may own the localization/Nav2 process group for a selected building/floor/map.
- A confirmed `ready` runtime context for the same map is reused by default.
- A fresh same-map `starting` runtime context is treated as an in-progress owner and API resume requests return `navigation_runtime_starting_reused` instead of forking another runtime.
- Stale `starting` contexts are allowed to be replaced after `navigation_resume_starting_context_ttl_sec`.
- `POST /api/v1/navigation/start` requires the requested map to be the floor's
  single active offline selection, revalidates its immutable identity and
  `current/` projection, and rejects a mismatched resident runtime.
- Offline selection freezes
  `building/floor/map/asset_epoch/asset_digest`; the floor-manager service
  validates the exact source bundle even when `current/` is absent, and the
  API refuses activation unless the response echoes the same identity and
  source paths.
- App editor-map selection is local UI state and never starts or reuses a
  runtime.
- A `resume_navigation=false` request for the exact confirmed `ready` runtime
  map may return `runtime_map_already_selected` as a zero-side-effect
  compatibility no-op. It does not enter this runtime-resume path.

## Hardware Validation

On Jetson, reboot with last-map autostart enabled and verify controlled startup
reuse leaves one resident runtime process. An App
`resume_navigation=true` request must be rejected without creating a second
`/tmp/njrh_navigation_resume.log` owner or touching `map_server`,
`robot_localization_bridge`, AMCL, or Nav2. Separately, select one exact map
offline, call `/api/v1/navigation/start` twice, and verify both calls converge
on one process and one matching confirmed runtime context before any goal is
sent.
