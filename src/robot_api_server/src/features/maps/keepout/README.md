# Keepout maps

Owns keepout semantic-layer IO, geometry validation, map-origin conversion,
conservative rasterization, revisioning, rollback, and runtime-effect proof
data structures.

It does not cancel Nav2, move the robot, switch floors, or restart a costmap.
The composition root supplies the bounded ROS adapter used for live proof.
