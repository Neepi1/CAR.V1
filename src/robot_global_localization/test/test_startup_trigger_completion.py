"""Exercise the startup client's completion callback without creating ROS entities."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


class StartupTriggerCompletion(unittest.TestCase):
    def run_status(self, **overrides):
        status = dict(last_explicit_relocalization_sequence=4, has_map_to_odom=True,
                      map_to_odom_publisher_owner='robot_localization_bridge',
                      safe_for_goal_start=False, correction_active=False,
                      current_sequence=8, target_sequence=8)
        status.update(overrides)
        samples = [status]
        callback = []
        node = SimpleNamespace(create_subscription=lambda _t, _n, cb, _q: callback.append(cb))
        ros = SimpleNamespace(ok=lambda: bool(samples),
                              spin_once=lambda *a, **k: callback[0](
                                  SimpleNamespace(data=json.dumps(samples.pop(0)))))
        modules = {
            'rclpy': ros,
            'rclpy.qos': SimpleNamespace(DurabilityPolicy=SimpleNamespace(VOLATILE=0),
                ReliabilityPolicy=SimpleNamespace(RELIABLE=1), QoSProfile=lambda **kw: kw),
            'robot_interfaces.srv': SimpleNamespace(TriggerLocalization=object),
            'std_msgs.msg': SimpleNamespace(String=object),
        }
        path = Path(__file__).resolve().parents[3] / 'scripts/jetson/runtime_overlay/scripts/call_global_localization_trigger.py'
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('startup_trigger_under_test', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.floor_handoff_requested = lambda: False
            return module.wait_for_accepted_bridge(node, 3)

    def test_applied_localization_does_not_require_navigation_permission(self):
        self.assertEqual(self.run_status(), 0)

    def test_correction_still_active_does_not_complete(self):
        self.assertEqual(self.run_status(correction_active=True), 130)

    def test_old_sequence_does_not_complete(self):
        self.assertEqual(self.run_status(last_explicit_relocalization_sequence=3), 130)

    def test_wrong_tf_owner_does_not_complete(self):
        self.assertEqual(self.run_status(map_to_odom_publisher_owner='other'), 130)

    def test_unapplied_target_does_not_complete(self):
        self.assertEqual(self.run_status(current_sequence=7), 130)


if __name__ == '__main__':
    unittest.main()
