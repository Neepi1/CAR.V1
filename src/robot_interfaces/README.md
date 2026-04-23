# robot_interfaces

Shared ROS 2 interfaces for the first-round delivery robot stack.

## Scope

- Floor asset apply and localization trigger services
- Mission and elevator action contracts
- Health and mapping status messages

## Notes

- Interfaces are intentionally small so wrappers can compile against a stable contract early.
- Runtime ownership still follows the canonical TF policy in the repository root docs.
