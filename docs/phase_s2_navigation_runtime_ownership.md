# Phase S2 Navigation Runtime Ownership

## Problem

After boot, the systemd resident navigation runtime can already be starting or running while the App sends `POST /api/v1/floors/switch` with `resume_navigation=true`. Starting a second runtime for the same building, floor, and map can make the new process clean up the existing localization stack, which then causes the first resident runtime to stop Nav2.

## Contract

- Only one resident navigation runtime may own the localization/Nav2 process group for a selected building/floor/map.
- A confirmed `ready` runtime context for the same map is reused by default.
- A fresh same-map `starting` runtime context is treated as an in-progress owner and API resume requests return `navigation_runtime_starting_reused` instead of forking another runtime.
- Stale `starting` contexts are allowed to be replaced after `navigation_resume_starting_context_ttl_sec`.

## Hardware Validation

On Jetson, reboot with last-map autostart enabled, open the App during startup, and trigger floor resume twice. Expected result: one resident runtime process, no second `/tmp/njrh_navigation_resume.log` process killing `map_server`, `robot_localization_bridge`, AMCL, or Nav2.
