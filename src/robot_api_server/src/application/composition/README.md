# Application composition

`ApplicationCompositionModule` is the single object graph and lifecycle root
for the API process. It declares configuration through the dedicated
configuration modules, constructs every feature aggregate, connects their
narrow ports, completes late dependencies, starts the HTTP gateway, and shuts
the graph down in dependency-safe order.

This directory owns integration wiring only. Navigation, mapping,
localization, maps, floor switching, elevator, docking, power, safety, teleop,
subscriptions, status rendering, HTTP transport, and process-loop behavior
remain implemented by their respective modules.
