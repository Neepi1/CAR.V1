# Navigation runtime

Owns the bounded localization-bridge settling transaction and navigation
cancel runtime state/policy. Bridge waits preserve the existing distinction
between ordinary goal-start readiness and stricter final-pose verification,
including the AMCL static-standby exception and zero-command cadence.

It does not own ROS subscriptions, action clients, process launch, velocity
arbitration, or the `map->odom` transform. Those are injected by the
composition root.
