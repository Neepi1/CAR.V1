# Isaac relocalization tuning

The Ranger Mini 3 JT128 occupancy-grid-localizer profile uses:

```yaml
max_beam_error: 0.50
```

This matches the upstream Isaac ROS Occupancy Grid Localizer default. The value
caps the error contributed by each scan beam during scan-to-map comparison; it
does not change Nav2 costmaps, inflation, or collision clearance. All other
localizer parameters remain frozen for this change.

Hardware validation must be performed while stationary. Record the returned
pose, latency, and whether the physically correct map basin wins across several
fresh scans at the previously ambiguous F2 location and at ordinary reference
locations. A parameter-only result is not proof against geometric aliasing;
coarse pose/yaw admission remains the production safeguard for ambiguous global
matches.
