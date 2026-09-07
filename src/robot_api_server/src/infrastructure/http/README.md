# HTTP infrastructure module

This directory owns the complete public HTTP gateway boundary in two layers.

`ApiGatewayConfigurationModule` uniquely declares the `host`, `port`,
`api_token`, and `max_http_connections` ROS parameters and emits the existing
`ApiGatewayModuleConfig`.

`ApiGatewayModule` owns gateway-wide runtime policy and lifecycle:

- host/port/token/connection runtime normalization;
- `ROBOT_API_TOKEN` fallback when the ROS parameter is empty;
- listener start/stop and WebSocket-session shutdown ordering;
- unauthenticated CORS `OPTIONS` handling and shared token validation;
- access/event logging and active/max connection observations;
- the exact `/api/v1/openapi` endpoint catalog;
- delegation to one authenticated business-route port and one socket-route
  port.

`HttpServer` remains the generic transport mechanism. It owns the listening
socket, bounded worker pool, HTTP request framing and size limits,
loopback-peer observation, response framing/CORS headers, connection
accounting, and synchronous socket-route handoff. It has no ROS or robot
business dependency.

The directory also owns the transport-level request/response models, HTTP
parsing and framing helpers, WebSocket primitives, and small JSON
parsing/escaping helpers shared by the gateway. `websocket_transport.cpp` owns
upgrade-header validation, the 101 handshake, receive timeout setup, masked
client-frame decoding, and unmasked server-frame encoding. It has no teleop
motion-admission or velocity knowledge.

Robot endpoint precedence and map/navigation/docking/elevator decisions remain
outside this directory in `application/routing`; its authenticated route is
injected through the composition root. Connection ownership is explicit: the
server closes every accepted descriptor after the normal route or socket route
returns.
