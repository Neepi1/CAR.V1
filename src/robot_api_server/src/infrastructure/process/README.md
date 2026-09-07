# Process infrastructure

Owns the ROS process bootstrap, the single-worker deferred queue, and Linux
fd/session/process-group helpers used by the API composition root.

`robot_api_process` initializes ROS, creates the minimal concrete API node and
its `ApplicationCompositionModule`, runs the single-threaded executor with the
established transient action-client retry, and performs process shutdown. The
top-level `robot_api_server_node.cpp` only calls this entry point.

It does not decide which runtime should start or stop and contains no robot
mission, navigation, docking, elevator, or safety policy.
