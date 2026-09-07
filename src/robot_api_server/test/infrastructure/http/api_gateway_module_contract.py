#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
NODE = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "composition"
    / "application_composition_module.cpp"
)
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "infrastructure"
    / "http"
    / "api_gateway_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "infrastructure" / "http" / "api_gateway_module.cpp"
ROUTER_SOURCE = (
    PACKAGE_ROOT / "src" / "application" / "routing" / "application_router_module.cpp"
)
ROUTER_WIRING = (
    PACKAGE_ROOT / "src" / "application" / "routing" / "application_router_wiring.cpp"
)
SYSTEM_STATUS_WIRING = (
    PACKAGE_ROOT / "src" / "features" / "system_status" / "system_status_wiring.cpp"
)


def main() -> None:
    assert HEADER.exists(), "ApiGatewayModule public boundary is missing"
    assert SOURCE.exists(), "ApiGatewayModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    router = ROUTER_SOURCE.read_text(encoding="utf-8")
    router_wiring = ROUTER_WIRING.read_text(encoding="utf-8")
    system_status_wiring = SYSTEM_STATUS_WIRING.read_text(encoding="utf-8")

    assert "class ApiGatewayModule" in header
    assert "void start()" in header
    assert "void stop()" in header
    assert "bool token_allowed(const HttpRequest & request) const" in header
    assert "std::optional<HttpResponse> handle_metadata" in header
    assert "int active_connections() const noexcept" in header

    assert 'std::getenv("ROBOT_API_TOKEN")' in source
    assert 'request.method == "OPTIONS"' in source
    assert "missing or invalid X-Robot-Token" in source
    assert "HttpServerCallbacks" in source
    assert "HttpServerOptions" in source
    assert "log_http_request" in source
    assert "/api/v1/openapi" in source
    assert "WS /ws/v1/teleop" in source
    assert "POST /api/v1/mapping/3d/start" in source

    assert 'infrastructure/http/api_gateway_module.hpp' in node
    assert 'infrastructure/http/http_server.hpp' not in node
    assert "std::unique_ptr<ApiGatewayModule> api_gateway_module_;" in node
    assert "api_gateway_module_->start();" in node
    assert "api_gateway_module_->stop();" in node
    assert "gateway->handle_metadata(request)" in router_wiring
    assert "dependencies.api_gateway->active_connections()" in system_status_wiring
    assert "api_gateway_module_->active_connections()" not in node

    for ownership_marker in (
        "void start_server()",
        "void stop_server()",
        "void send_response(",
        "void log_http_request(",
        "bool token_allowed(",
        "HttpResponse handle_openapi(",
        "std::unique_ptr<HttpServer>",
        "HttpServerCallbacks",
        "HttpServerOptions",
        "HttpServerLogLevel",
        "max_http_connections_",
        "api_token_",
        "std::atomic<bool> running_",
        'std::getenv("ROBOT_API_TOKEN")',
    ):
        assert ownership_marker not in node, ownership_marker

    # The application router owns business-module precedence, while transport
    # preflight (OPTIONS/auth) belongs exclusively to the gateway.
    assert 'request.method == "OPTIONS"' not in router
    assert "missing or invalid X-Robot-Token" not in router
    assert "HttpResponse route(" not in node


if __name__ == "__main__":
    main()
