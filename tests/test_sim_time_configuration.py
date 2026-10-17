from __future__ import annotations

import os
import unittest
from unittest.mock import patch

from hardware.navigation import NavigationController
from runtime_gateway import RobotRuntimeGateway


class SimTimeConfigurationTests(unittest.TestCase):
    def test_navigation_controller_uses_sim_time_only_for_sim_runtime(self):
        for runtime_id, expected in (("sim", True), ("SIM", True), ("real", False)):
            with self.subTest(runtime_id=runtime_id):
                with patch.dict(os.environ, {"LIBOT_RUNTIME_ID": runtime_id}):
                    self.assertEqual(NavigationController().use_sim_time, expected)

    def test_runtime_gateway_uses_sim_time_only_for_sim_runtime(self):
        for runtime_id, expected in (("sim", True), ("SIM", True), ("real", False)):
            with self.subTest(runtime_id=runtime_id):
                with patch.dict(os.environ, {"LIBOT_RUNTIME_ID": runtime_id}):
                    self.assertEqual(RobotRuntimeGateway().use_sim_time, expected)

    def test_navigation_waits_until_the_simulated_clock_is_nonzero(self):
        class FakeNode:
            nanoseconds = 0

            def get_clock(self):
                return self

            def now(self):
                return self

        class FakeRclpy:
            @staticmethod
            def ok():
                return True

            @staticmethod
            def spin_once(node, timeout_sec):
                node.nanoseconds = 1

        self.assertTrue(
            NavigationController._wait_for_sim_clock(
                FakeNode(), FakeRclpy(), timeout_sec=0.1
            )
        )


if __name__ == "__main__":
    unittest.main()
