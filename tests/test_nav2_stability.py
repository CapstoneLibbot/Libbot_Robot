from __future__ import annotations

import math
import sys
import time
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from runtime_gateway import RobotRuntimeGateway


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / "ros2_ws" / "src" / "libot_bringup"


def stable_samples(count: int = 5):
    return [
        {
            "x": 1.0 + index * 0.005,
            "y": -0.5,
            "yaw": 0.2 + index * 0.005,
            "cov_x": 0.01,
            "cov_y": 0.015,
            "cov_yaw": 0.01,
            "received_at": time.monotonic(),
        }
        for index in range(count)
    ]


class Nav2StabilityTests(unittest.TestCase):
    def test_lifecycle_summary_identifies_partially_started_stack(self):
        gateway = RobotRuntimeGateway()
        states = dict.fromkeys(gateway.NAVIGATION_NODES, 1)
        states["controller_server"] = 3
        states["bt_navigator"] = 2

        summary = gateway._format_lifecycle_states(states)

        self.assertIn("controller_server=active", summary)
        self.assertIn("bt_navigator=inactive", summary)
        self.assertIn("velocity_smoother=unconfigured", summary)

    def test_amcl_stability_requires_five_fresh_and_bounded_samples(self):
        samples = stable_samples()
        self.assertFalse(RobotRuntimeGateway._amcl_window_is_stable(samples[:4]))
        self.assertTrue(RobotRuntimeGateway._amcl_window_is_stable(samples))

        high_covariance = stable_samples()
        high_covariance[-1]["cov_x"] = 0.05
        self.assertFalse(RobotRuntimeGateway._amcl_window_is_stable(high_covariance))

        spread = stable_samples()
        spread[-1]["x"] += 0.2
        self.assertFalse(RobotRuntimeGateway._amcl_window_is_stable(spread))

        yaw_spread = stable_samples()
        yaw_spread[-1]["yaw"] += 0.3
        self.assertFalse(RobotRuntimeGateway._amcl_window_is_stable(yaw_spread))

        invalid = stable_samples()
        invalid[0]["cov_yaw"] = math.nan
        self.assertFalse(RobotRuntimeGateway._amcl_window_is_stable(invalid))

    def test_amcl_callback_records_fresh_samples_without_marking_localized(self):
        gateway = RobotRuntimeGateway()
        gateway.navigation_mode_state = "LOCALIZING"
        gateway.localization_state = "UNLOCALIZED"
        gateway._localization_attempt_start_stamp_ns = 20_000_000_000

        def message(stamp_ns):
            return SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(
                        sec=stamp_ns // 1_000_000_000,
                        nanosec=stamp_ns % 1_000_000_000,
                    )
                ),
                pose=SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=1.0, y=2.0),
                        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    covariance=[0.01] * 36,
                ),
            )

        gateway._on_amcl_pose(message(19_000_000_000))
        self.assertEqual(len(gateway._amcl_samples), 0)
        gateway._on_amcl_pose(message(21_000_000_000))
        self.assertEqual(len(gateway._amcl_samples), 1)
        self.assertEqual(gateway.localization_state, "UNLOCALIZED")

        gateway.navigation_mode_state = "READY"
        gateway.localization_state = "LOCALIZED"
        gateway._on_amcl_pose(message(22_000_000_000))
        self.assertEqual(len(gateway._amcl_samples), 1)
        self.assertEqual(gateway.localization_state, "LOCALIZED")

    def test_stable_amcl_requests_nomotion_and_activates_navigation(self):
        calls = []

        class FakeFuture:
            def add_done_callback(self, callback):
                callback(self)

            def result(self):
                return SimpleNamespace(success=True)

        class Empty:
            class Request:
                pass

        class LifecycleRequest:
            STARTUP = 0
            PAUSE = 1
            RESUME = 2

            def __init__(self):
                self.command = None

        class ManageLifecycleNodes:
            Request = LifecycleRequest

        std_srvs = types.ModuleType("std_srvs")
        std_srvs_srv = types.ModuleType("std_srvs.srv")
        std_srvs_srv.Empty = Empty
        nav2_msgs = types.ModuleType("nav2_msgs")
        nav2_msgs_srv = types.ModuleType("nav2_msgs.srv")
        nav2_msgs_srv.ManageLifecycleNodes = ManageLifecycleNodes

        class FakeClient:
            def service_is_ready(self):
                return True

            def wait_for_service(self, timeout_sec):
                return True

            def call_async(self, request):
                calls.append(request)
                return FakeFuture()

        gateway = RobotRuntimeGateway()
        gateway._running = True
        gateway.navigation_mode_state = "LOCALIZING"
        gateway._localization_attempt_id = 8
        gateway._amcl_samples.extend(stable_samples())
        gateway._last_scan_monotonic = time.monotonic()
        gateway._nomotion_update_client = FakeClient()
        gateway._navigation_lifecycle_client = FakeClient()
        gateway._navigation_map_path = "/ros2_ws/maps/library.yaml"
        gateway._lookup_map_pose = lambda: {"x": 1.0, "y": -0.5, "yaw": 0.2}
        ros_now_ns = 10_000_000_000
        gateway._node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=ros_now_ns)
            )
        )
        gateway._last_scan_ros_stamp_ns = ros_now_ns
        gateway._action_client = SimpleNamespace(wait_for_server=lambda timeout_sec: True)
        gateway._wait_for_navigation_nodes_active = lambda timeout_sec: True
        with patch.dict(sys.modules, {
            "std_srvs": std_srvs,
            "std_srvs.srv": std_srvs_srv,
            "nav2_msgs": nav2_msgs,
            "nav2_msgs.srv": nav2_msgs_srv,
        }):
            gateway._localization_worker(8)
            gateway.navigation_mode_state = "LOCALIZING"
            gateway._navigation_lifecycle_paused = True
            gateway._localization_attempt_id = 9
            gateway._amcl_samples.clear()
            gateway._amcl_samples.extend(stable_samples())
            gateway._last_scan_monotonic = time.monotonic()
            gateway._localization_worker(9)

        lifecycle_requests = [call for call in calls if hasattr(call, "command")]
        self.assertGreaterEqual(len(calls), 4)
        self.assertEqual(lifecycle_requests[0].command, LifecycleRequest.STARTUP)
        self.assertEqual(lifecycle_requests[1].command, LifecycleRequest.RESUME)
        self.assertEqual(gateway.localization_state, "LOCALIZED")
        self.assertEqual(gateway.navigation_mode_state, "READY")
        self.assertTrue(gateway._navigation_lifecycle_started)
        self.assertFalse(gateway._navigation_lifecycle_paused)

    def test_unstable_amcl_tf_or_scan_leaves_navigation_in_error(self):
        cases = (
            ("covariance", True, True),
            ("spread", True, True),
            ("missing_tf", True, False),
            ("stale_scan", False, True),
        )
        ros_now_ns = 100_000_000_000
        for failure, scan_fresh, tf_available in cases:
            with self.subTest(failure=failure):
                gateway = RobotRuntimeGateway()
                gateway._running = True
                gateway.AMCL_LOCALIZATION_TIMEOUT_SEC = 0.005
                gateway.navigation_mode_state = "LOCALIZING"
                gateway._localization_attempt_id = 3
                samples = stable_samples()
                if failure == "covariance":
                    samples[0]["cov_y"] = 0.2
                elif failure == "spread":
                    samples[-1]["y"] = 1.0
                gateway._amcl_samples.extend(samples)
                gateway._last_scan_monotonic = (
                    time.monotonic() if scan_fresh else time.monotonic() - 2.0
                )
                gateway._last_scan_ros_stamp_ns = (
                    ros_now_ns if scan_fresh else ros_now_ns - 2_000_000_000
                )
                gateway._node = SimpleNamespace(
                    get_clock=lambda: SimpleNamespace(
                        now=lambda: SimpleNamespace(nanoseconds=ros_now_ns)
                    )
                )
                gateway._lookup_map_pose = lambda: ({"x": 0, "y": 0, "yaw": 0} if tf_available else None)
                stopped = []
                gateway._publish_stop = lambda: stopped.append(True)
                gateway._localization_worker(3)
                self.assertEqual(gateway.navigation_mode_state, "ERROR")
                self.assertEqual(gateway.error, "AMCL_NOT_STABLE")
                self.assertEqual(gateway.localization_state, "UNLOCALIZED")
                self.assertFalse(gateway._navigation_lifecycle_started)
                self.assertTrue(stopped)

    def test_retry_initial_pose_clears_previous_amcl_window(self):
        class FakeClock:
            def now(self):
                return self

            def to_msg(self):
                return object()

        class FakePoseWithCovarianceStamped:
            def __init__(self):
                self.header = SimpleNamespace(frame_id="", stamp=None)
                self.pose = SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0),
                        orientation=SimpleNamespace(z=0.0, w=1.0),
                    ),
                    covariance=[0.0] * 36,
                )

        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.PoseWithCovarianceStamped = FakePoseWithCovarianceStamped

        class FakeProcess:
            @staticmethod
            def poll():
                return None

        gateway = RobotRuntimeGateway()
        gateway._node = SimpleNamespace(get_clock=lambda: FakeClock())
        gateway._initial_pose_pub = SimpleNamespace(publish=lambda _: None)
        gateway._navigation_process = FakeProcess()
        gateway._navigation_map_path = "/ros2_ws/maps/library.yaml"
        gateway._running = False
        gateway.navigation_mode_state = "ERROR"
        gateway._amcl_samples.extend(stable_samples())
        with patch.dict(sys.modules, {
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
        }):
            result = gateway._set_initial_pose({"x": 1.0, "y": 2.0, "yaw": 0.5})

        self.assertEqual(result["status"], "accepted")
        self.assertEqual(gateway.navigation_mode_state, "LOCALIZING")
        self.assertEqual(gateway.localization_state, "LOCALIZING")
        self.assertEqual(gateway.navigation_state, "IDLE")
        self.assertEqual(len(gateway._amcl_samples), 0)

    def test_progress_feedback_restarts_timeout(self):
        created = []

        class FakeTimer:
            def __init__(self, interval, callback, args):
                self.interval = interval
                self.callback = callback
                self.args = args
                self.started = False
                self.cancelled = False
                created.append(self)

            def start(self):
                self.started = True

            def cancel(self):
                self.cancelled = True

        gateway = RobotRuntimeGateway()
        gateway.active_goal_id = "goal-1"
        gateway._goal_timeout_generation = 11
        gateway._goal_timeout_sec = 180.0
        gateway._goal_best_distance_remaining = 10.0
        gateway._goal_progress_pose = (0.0, 0.0)
        gateway._goal_timeout_timer = FakeTimer(180.0, lambda: None, ())
        old_timer = gateway._goal_timeout_timer
        created.clear()
        with patch("runtime_gateway.threading.Timer", FakeTimer):
            gateway._on_feedback(
                SimpleNamespace(feedback=SimpleNamespace(distance_remaining=9.94)),
                "goal-1",
                11,
            )

        self.assertTrue(old_timer.cancelled)
        self.assertEqual(len(created), 1)
        self.assertTrue(created[0].started)
        self.assertEqual(created[0].args, ("goal-1", 11))
        self.assertEqual(gateway._goal_best_distance_remaining, 9.94)

        gateway.pose["x"] = 0.06
        with patch("runtime_gateway.threading.Timer", FakeTimer):
            gateway._on_feedback(
                SimpleNamespace(feedback=SimpleNamespace(distance_remaining=9.94)),
                "goal-1",
                11,
            )
        self.assertTrue(created[0].cancelled)
        self.assertEqual(len(created), 2)
        self.assertEqual(created[1].args, ("goal-1", 11))

    def test_stationary_goal_times_out_and_stale_timer_cannot_fail_new_goal(self):
        cancelled = []
        stopped = []
        gateway = RobotRuntimeGateway()
        gateway.active_goal_id = "new-goal"
        gateway._goal_timeout_generation = 22
        gateway.navigation_state = "NAVIGATING"
        gateway._goal_handle = SimpleNamespace(cancel_goal_async=lambda: cancelled.append(True))
        gateway._publish_stop = lambda: stopped.append(True)

        gateway._on_goal_timeout("old-goal", 21)
        self.assertEqual(gateway.active_goal_id, "new-goal")
        self.assertEqual(stopped, [])
        gateway._on_feedback(
            SimpleNamespace(feedback=SimpleNamespace(distance_remaining=0.0)),
            "old-goal",
            21,
        )
        gateway._on_navigation_result(
            SimpleNamespace(result=lambda: SimpleNamespace(status=4)), "old-goal", 21
        )
        self.assertEqual(gateway.active_goal_id, "new-goal")
        self.assertEqual(gateway.navigation_state, "NAVIGATING")

        gateway._on_goal_timeout("new-goal", 22)
        self.assertIsNone(gateway.active_goal_id)
        self.assertEqual(gateway.navigation_state, "FAILED")
        self.assertEqual(gateway.error, "NO_PROGRESS_TIMEOUT")
        self.assertTrue(cancelled)
        self.assertTrue(stopped)

    def test_aborted_navigation_preserves_nav2_error_details(self):
        gateway = RobotRuntimeGateway()
        gateway.active_goal_id = "goal-1"
        gateway._goal_timeout_generation = 7
        wrapped = SimpleNamespace(
            status=6,
            result=SimpleNamespace(error_code=9003, error_msg="Failed to make progress"),
        )

        gateway._on_navigation_result(
            SimpleNamespace(result=lambda: wrapped), "goal-1", 7
        )

        self.assertEqual(gateway.navigation_state, "FAILED")
        self.assertEqual(
            gateway.error,
            "NAVIGATION_FAILED: code=9003; Failed to make progress",
        )
        self.assertIsNone(gateway.active_goal_id)


class Nav2ConfigurationTests(unittest.TestCase):
    def test_custom_behavior_trees_replan_while_following_with_limited_recovery(self):
        for name, compute_tag, goal_input in (
            ("navigate_to_pose.xml", "ComputePathToPose", "goal"),
            ("navigate_through_poses.xml", "ComputePathThroughPoses", "goals"),
        ):
            with self.subTest(tree=name):
                root = ET.parse(BRINGUP / "behavior_trees" / name).getroot()
                self.assertEqual(root.attrib["BTCPP_format"], "4")
                self.assertEqual(len(root.findall(f".//{compute_tag}")), 1)
                self.assertEqual(len(root.findall(".//FollowPath")), 1)
                self.assertEqual(len(root.findall(".//PipelineSequence")), 1)
                rate_controller = root.find(".//PipelineSequence/RateController")
                self.assertIsNotNone(rate_controller)
                self.assertEqual(rate_controller.attrib["hz"], "1.0")
                compute = root.find(".//" + compute_tag)
                follow = root.find(".//FollowPath")
                self.assertEqual(compute.attrib[goal_input], "{" + goal_input + "}")
                self.assertEqual(compute.attrib["error_code_id"], "{compute_path_error_code}")
                self.assertEqual(follow.attrib["error_code_id"], "{follow_path_error_code}")
                recovery = root.find(".//RecoveryNode")
                self.assertEqual(recovery.attrib["number_of_retries"], "2")
                tags = [element.tag for element in root.iter()]
                for forbidden in ("Spin", "BackUp"):
                    self.assertNotIn(forbidden, tags)
                self.assertEqual(len(root.findall(".//Wait")), 1)
                costmap_services = {
                    node.attrib.get("service_name")
                    for node in root.findall(".//ClearEntireCostmap")
                }
                self.assertEqual(costmap_services, {
                    "local_costmap/clear_entirely_local_costmap",
                    "global_costmap/clear_entirely_global_costmap",
                })
                self.assertEqual(
                    [node.attrib["wait_duration"] for node in root.findall(".//Wait")],
                    ["1.0"],
                )

    def test_launch_uses_installed_trees_and_holds_navigation_lifecycle(self):
        launch = (BRINGUP / "launch" / "navigation.launch.py").read_text(encoding="utf-8")
        cmake = (BRINGUP / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('"behavior_trees", "navigate_to_pose.xml"', launch)
        self.assertIn('"behavior_trees", "navigate_through_poses.xml"', launch)
        self.assertIn('"autostart": False', launch)
        self.assertIn('"bt_navigator"', launch)
        self.assertIn("DIRECTORY launch config behavior_trees", cmake)
        config = (BRINGUP / "config" / "nav2.yaml").read_text(encoding="utf-8")
        self.assertIn("- navigate_to_pose", config)
        self.assertIn("- navigate_through_poses", config)
        self.assertIn(
            "default_nav_to_pose_bt_xml: /ros2_ws/src/libot_bringup/behavior_trees/navigate_to_pose.xml",
            config,
        )
        self.assertIn(
            "default_nav_through_poses_bt_xml: /ros2_ws/src/libot_bringup/behavior_trees/navigate_through_poses.xml",
            config,
        )

    def test_nav2_tuning_matches_conservative_motion_policy(self):
        config = (BRINGUP / "config" / "nav2.yaml").read_text(encoding="utf-8")
        expected = (
            "publish_zero_velocity: true",
            "plugin: nav2_controller::PoseProgressChecker",
            "required_movement_radius: 0.05",
            "required_movement_angle: 0.10",
            "movement_time_allowance: 30.0",
            "failure_tolerance: 1.5",
            "stateful: true",
            "xy_goal_tolerance: 0.10",
            "yaw_goal_tolerance: 0.20",
            "rotate_to_heading_angular_vel: 0.8",
            "rotate_to_heading_min_angle: 0.524",
            "max_angular_accel: 4.0",
            "max_rotational_vel: 0.8",
            "min_rotational_vel: 0.2",
            "rotational_acc_lim: 2.0",
            "- -4.0",
            "allow_reversing: false",
            "allow_unknown: false",
        )
        for value in expected:
            with self.subTest(setting=value):
                self.assertIn(value, config)
        behavior_config = config.split("behavior_server:", 1)[1].split("waypoint_follower:", 1)[0]
        self.assertIn("- wait", behavior_config)
        self.assertNotRegex(behavior_config, r"(?m)^\s*-\s*(spin|backup)\s*$")


if __name__ == "__main__":
    unittest.main()
