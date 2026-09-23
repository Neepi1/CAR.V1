# Collision monitor geometry

## Scan-mask update (2026-09-21; restart required for activation)

The user-requested Scan mask is now X +/-0.39 m, Y +/-0.36 m. StopZone itself
is unchanged. The central sides have no remaining scan-visible band inside
StopZone; front/rear retain 8 cm. Ultrasound is not integrated. See
[scope and verification limits](body_geometry_alignment.md#scan-mask-width-update-2026-09-21).

## Current local candidate (2026-09-19, not deployed)

The user confirmed that the existing StopZone (`+/-0.47 / +/-0.36 m`) is the
physical body. The raw global/local footprint now matches it; retained padding
makes the planning/published footprint `+/-0.50 / +/-0.39 m`. StopZone, scan mask,
thresholds, SlowZone and Approach settings are not changed. FootprintApproach
does consume the larger published footprint, so its predicted action may change.
See [geometry alignment and verification limits](body_geometry_alignment.md).

## Previous candidate (2026-09-08, geometry interpretation superseded)

StopZone is now `x=+/-0.47 m`, `y=+/-0.36 m` (full `0.94 x 0.72 m`).
The self mask and padded costmap footprint remain `0.39/0.28 m`, leaving an
8 cm visible band on every side. MPPI separately prefers `0.52/0.41 m` through
a finite scoring critic, not enlarged footprint clearing. The two-return
threshold, SlowZone, FootprintApproach and velocity chain remain unchanged.
See [planning clearance](planning_clearance.md) for implementation, fallback,
scope and required low-speed hardware verification. This edit is not proof
of safe stopping at the configured 1.2 m/s cruise speed.

## Previous configuration (superseded)

On 2026-09-07, the user requested StopZone half extents of `x=0.42 m`,
`y=0.28 m`, replacing `0.50 m`, `0.40 m` in both Nav2 configurations.
Coordinates are relative to `base_link`: front/rear `+/-0.42 m`,
left/right `+/-0.28 m`; the full rectangle is `0.84 x 0.56 m`.

The configured physical body has half extents `0.36 x 0.25 m` and costmap
padding `0.03 m`. StopZone therefore extends 6 cm longitudinally and 3 cm
laterally beyond the unpadded body; it covers the padded `0.39 x 0.28 m` body.
This hard-stop polygon is no longer rounded outward to include MPPI's soft
collision margin. MPPI, inflation, body geometry, SlowZone, FootprintApproach,
and the StopZone two-return threshold remain unchanged.

The smaller polygon triggers later and does not by itself prove adequate braking
distance or solve route selection around moving obstacles. Hardware validation
remains necessary for front, rear, lateral approaches and obstacle removal,
recording which collision-monitor zone acted. No motion test is implied by the
configuration edit; perform it only with explicit operator authorization.

Historical tests covered both configurations, exact vertices, padded-body containment and
SlowZone containment. Preserve unrelated local/Jetson configuration differences
when deploying this geometry-only change. Apply at the next authorized full
`njrh-runtime.service` restart; do not restart collision_monitor separately or
treat a successful parameter write as proof that its polygon was rebuilt.
