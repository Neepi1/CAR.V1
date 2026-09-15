"""Offline constructor contract, not proof of ROS discovery/service success.

Execute the actual Python main/heredoc up to rclpy.create_node, then stop before
any node or ROS request exists. Hardware/isolation tests own ROS behavior.
"""

import re
import sys
import types

import pytest

from test_amcl_startup_sequence import SCRIPTS, shell_function


class ConstructorReached(BaseException):
    """Stop at the external constructor without pretending ROS succeeded."""

    def __init__(self, args, kwargs):
        self.args_received = args
        self.kwargs_received = kwargs


CASES = [
    ("seed_amcl_initial_pose", "amcl_seed_initial_pose_client"),
    ("wait_for_scan_admission_status_ready", "amcl_scan_admission_ready_waiter"),
    ("wait_for_amcl_pose_fresh", "amcl_pose_fresh_waiter"),
    ("amcl_nomotion_update_probe.py", "amcl_nomotion_update_probe"),
    ("call_global_localization_trigger.py", "startup_global_localization_trigger_client"),
]


@pytest.mark.parametrize("entry,node_name", CASES)
def test_startup_clients_disable_unneeded_ros_endpoints(monkeypatch, entry, node_name):
    events = []

    def initialize(*args, **kwargs):
        events.append("init")

    def observe_constructor(*args, **kwargs):
        events.append("create_node")
        raise ConstructorReached(args, kwargs)

    rclpy = types.ModuleType("rclpy")
    rclpy.init = initialize
    rclpy.create_node = observe_constructor
    # The no-motion main's finally checks whether cleanup is needed. No actual
    # context was initialized, so no shutdown or service outcome is simulated.
    rclpy.ok = lambda: False
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    for module_name, class_names in {
        "rclpy.node": ("Node",),
        "std_srvs.srv": ("Trigger", "Empty"),
        "std_msgs.msg": ("String",),
        "geometry_msgs.msg": ("PoseWithCovarianceStamped",),
        "robot_interfaces.srv": ("TriggerLocalization",),
    }.items():
        module = types.ModuleType(module_name)
        for class_name in class_names:
            setattr(module, class_name, type(class_name, (), {}))
        monkeypatch.setitem(sys.modules, module_name, module)

    namespace = {"__name__": "startup_constructor_contract"}
    if entry.endswith(".py"):
        path = SCRIPTS / entry
        monkeypatch.setattr(sys, "argv", [entry, "--reason", "offline-constructor-contract"]
                            if entry == "call_global_localization_trigger.py" else [entry])
        exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"), namespace)
        execute = namespace["main"]
    else:
        heredoc = re.search(r"<<'PY'[^\n]*\n(.*?)\nPY\n", shell_function(entry), re.S)
        assert heredoc is not None, f"Python client heredoc missing: {entry}"
        code = compile(heredoc.group(1), f"run_amcl_shadow_localization.sh:{entry}", "exec")
        monkeypatch.setattr(sys, "argv", ["-", "/fixture/unused", "8", "8", str(SCRIPTS)])

        def execute():
            exec(code, namespace)

    with pytest.raises(ConstructorReached) as observed:
        execute()

    assert events == ["init", "create_node"]
    assert observed.value.args_received == (node_name,)
    assert observed.value.kwargs_received == {
        "enable_rosout": False,
        "start_parameter_services": False,
    }
