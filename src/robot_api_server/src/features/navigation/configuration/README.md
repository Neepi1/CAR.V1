# Navigation configuration

`NavigationConfigurationModule` is the construction-time owner of all 112
navigation ROS parameter declarations used by the API gateway. It preserves
the established defaults, clamps, dependent bounds, compatibility parameters,
and warning behavior, then composes the existing `NavigationModuleConfig`,
`NavigationTerminalRuntimeConfig`, `NavigationGoalExecutionConfig`, and
`NavigationGoalExecutorConfig` objects.

Shared service timeout, action/status names, map context, TF frame names, pose
freshness, and predock lateral-control values are explicit inputs owned by
neighboring modules. The AMCL no-motion service name is returned as an explicit
localization projection rather than retained as duplicate root-node state.

This module declares configuration only. It does not submit or cancel a Nav2
goal, publish velocity or Ranger mode, trigger localization, alter TF, add a
gate, or change ordinary-navigation, elevator, or docking acceptance policy.
