# Shared runtime configuration

`RuntimeConfigurationModule` uniquely declares the three application-wide
bootstrap parameters:

- the Nav2 `NavigateToPose` action name;
- its action-status topic; and
- the base ROS service timeout.

It returns one immutable construction-time projection consumed by navigation,
docking, elevator, localization, floor switch, and system status. It performs
no ROS request, graph probe, process operation, route handling, or motion
command, and it does not normalize the deployed values. Feature-specific
configuration modules remain the owners of their own parameters and derived
bounds.
