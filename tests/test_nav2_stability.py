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

import yaml

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
    def test_custom_behavior_trees_wait_replan_and_turn_safely(self):
        for name, compute_tag, goal_input in (
            ("navigate_to_pose.xml", "ComputePathToPose", "goal"),
            ("navigate_through_poses.xml", "ComputePathThroughPoses", "goals"),
        ):
            with self.subTest(tree=name):
                root = ET.parse(BRINGUP / "behavior_trees" / name).getroot()
                self.assertEqual(root.attrib["BTCPP_format"], "4")
                computes = root.findall(f".//{compute_tag}")
                self.assertEqual(len(computes), 2)
                self.assertEqual(len(root.findall(".//FollowPath")), 1)
                self.assertEqual(len(root.findall(".//PipelineSequence")), 1)
                rate_controller = root.find(".//PipelineSequence/RateController")
                self.assertIsNotNone(rate_controller)
                self.assertEqual(rate_controller.attrib["hz"], "1.0")
                compute = computes[0]
                follow = root.find(".//FollowPath")
                self.assertEqual(compute.attrib[goal_input], "{" + goal_input + "}")
                self.assertEqual(compute.attrib["error_code_id"], "{compute_path_error_code}")
                self.assertEqual(follow.attrib["error_code_id"], "{follow_path_error_code}")
                recovery = root.find("./BehaviorTree/RecoveryNode")
                self.assertIsNotNone(recovery)
                self.assertEqual(recovery.attrib["number_of_retries"], "6")
                tags = [element.tag for element in root.iter()]
                self.assertIn("PipelineSequence", tags)
                self.assertIn("RateController", tags)
                self.assertIn("BackUp", tags)
                self.assertIn("Spin", tags)
                self.assertNotIn("SequenceWithMemory", tags)
                rate = root.find(".//RateController")
                self.assertIsNotNone(rate)
                self.assertEqual(rate.attrib["hz"], "1.0")
                backups = root.findall(".//BackUp")
                self.assertEqual(len(backups), 1)
                for backup in backups:
                    self.assertEqual(backup.attrib["backup_dist"], "0.20")
                    self.assertEqual(backup.attrib["backup_speed"], "0.12")
                    self.assertEqual(backup.attrib["time_allowance"], "4.0")
                    self.assertNotIn("disable_collision_checks", backup.attrib)
                controller_recovery = root.find(
                    ".//Sequence[@name='BackUpAndReplanOnControllerFailure']"
                )
                self.assertIsNotNone(controller_recovery)
                self.assertIsNotNone(controller_recovery.find("BackUp"))
                self.assertIsNotNone(controller_recovery.find("Wait"))
                self.assertIsNone(controller_recovery.find("ClearEntireCostmap"))
                forced_replan = controller_recovery.find(compute_tag)
                self.assertIsNotNone(forced_replan)
                self.assertEqual(forced_replan.attrib[goal_input], "{" + goal_input + "}")
                self.assertEqual(forced_replan.attrib["path"], "{path}")
                spins = root.findall(".//Fallback[@name='TurnToFreeSide']/Spin")
                self.assertEqual(len(spins), 2)
                self.assertEqual(
                    [spin.attrib["spin_dist"] for spin in spins],
                    ["1.5708", "-1.5708"],
                )
                self.assertTrue(all(spin.attrib["time_allowance"] == "6.0" for spin in spins))
                self.assertEqual(len(root.findall(".//Wait")), 5)
                self.assertEqual(len(root.findall(".//WouldAPlannerRecoveryHelp")), 2)
                self.assertEqual(len(root.findall(".//WouldAControllerRecoveryHelp")), 2)
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
                    ["0.5", "0.5", "0.5", "0.5", "2.0"],
                )

    def test_launch_uses_installed_trees_and_holds_navigation_lifecycle(self):
        launch = (BRINGUP / "launch" / "navigation.launch.py").read_text(encoding="utf-8")
        cmake = (BRINGUP / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('"behavior_trees", "navigate_to_pose.xml"', launch)
        self.assertIn('"behavior_trees", "navigate_through_poses.xml"', launch)
        self.assertIn('LaunchConfiguration("autostart_navigation")', launch)
        self.assertIn('"autostart": ParameterValue(', launch)
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

    def test_nav2_tuning_matches_replanning_motion_policy(self):
        config_text = (BRINGUP / "config" / "nav2.yaml").read_text(encoding="utf-8")
        config = yaml.safe_load(
            config_text
        )
        amcl = config["amcl"]["ros__parameters"]
        self.assertEqual(amcl["robot_model_type"], "nav2_amcl::OmniMotionModel")

        controller = config["controller_server"]["ros__parameters"]
        self.assertEqual(controller["controller_frequency"], 20.0)
        follow = controller["FollowPath"]
        self.assertEqual(
            follow["plugin"], "nav2_mppi_controller::MPPIController"
        )
        self.assertEqual(follow["motion_model"], "Omni")
        self.assertEqual(follow["model_dt"], 0.05)
        self.assertEqual(follow["time_steps"], 56)
        self.assertEqual(follow["batch_size"], 1000)
        self.assertFalse(follow["visualize"])
        self.assertEqual(follow["vx_max"], 0.40)
        self.assertEqual(follow["vx_min"], -0.40)
        self.assertEqual(follow["vy_max"], 0.40)
        self.assertEqual(follow["wz_max"], 1.0)
        self.assertIn("OmniConstraintCritic", follow["critics"])
        self.assertIn("MecanumCritic", follow["critics"])
        self.assertNotIn("PathAngleCritic", follow["critics"])
        self.assertNotIn("PreferForwardCritic", follow["critics"])
        self.assertTrue(follow["PathAlignCritic"]["use_path_orientations"])
        self.assertEqual(follow["MecanumCritic"]["vx_max"], 0.40)
        self.assertEqual(follow["MecanumCritic"]["vy_max"], 0.40)
        self.assertEqual(follow["MecanumCritic"]["wz_max"], 1.33)

        smoother = config["velocity_smoother"]["ros__parameters"]
        self.assertEqual(smoother["max_velocity"], [0.40, 0.40, 1.0])
        self.assertEqual(smoother["min_velocity"], [-0.40, -0.40, -1.0])

        expected = (
            "publish_zero_velocity: true",
            "plugin: nav2_controller::PoseProgressChecker",
            "required_movement_radius: 0.05",
            "required_movement_angle: 0.10",
            "movement_time_allowance: 5.0",
            "failure_tolerance: 5.0",
            "stateful: true",
            "xy_goal_tolerance: 0.10",
            "yaw_goal_tolerance: 0.20",
            "max_rotational_vel: 0.5",
            "min_rotational_vel: 0.2",
            "rotational_acc_lim: 2.0",
            "- -4.0",
            "allow_unknown: false",
        )
        for value in expected:
            with self.subTest(setting=value):
                self.assertIn(value, config_text)
        behavior_config = config_text.split("behavior_server:", 1)[1].split(
            "waypoint_follower:", 1
        )[0]
        self.assertIn("- spin", behavior_config)
        self.assertIn("- backup", behavior_config)
        self.assertIn("- wait", behavior_config)
        self.assertIn("plugin: nav2_behaviors::Spin", behavior_config)
        self.assertIn("plugin: nav2_behaviors::BackUp", behavior_config)

    def test_sonar_range_layer_marks_local_and_global_costmaps(self):
        config = yaml.safe_load(
            (BRINGUP / "config" / "nav2.yaml").read_text(encoding="utf-8")
        )
        global_params = config["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertEqual(global_params["update_frequency"], 4.0)
        self.assertEqual(global_params["publish_frequency"], 2.0)
        expected_topics = [
            "/range/front",
            "/range/left",
            "/range/right",
            "/range/rear",
        ]
        expected_plugins = {
            "local_costmap": ["obstacle_layer", "range_layer", "inflation_layer"],
            "global_costmap": [
                "static_layer",
                "obstacle_layer",
                "range_layer",
                "inflation_layer",
            ],
        }

        for costmap_name, plugins in expected_plugins.items():
            with self.subTest(costmap=costmap_name):
                params = config[costmap_name][costmap_name]["ros__parameters"]
                self.assertEqual(params["plugins"], plugins)
                layer = params["range_layer"]
                self.assertEqual(
                    layer["plugin"], "nav2_costmap_2d::RangeSensorLayer"
                )
                self.assertTrue(layer["enabled"])
                self.assertEqual(layer["topics"], expected_topics)
                self.assertEqual(layer["input_sensor_type"], "VARIABLE")
                self.assertTrue(layer["clear_on_max_reading"])
                self.assertEqual(layer["clear_threshold"], 0.2)
                self.assertEqual(layer["mark_threshold"], 0.8)
                self.assertEqual(layer["no_readings_timeout"], 0.0)

    def test_measured_footprint_and_collision_monitor_velocity_chain(self):
        nav2_config = yaml.safe_load(
            (BRINGUP / "config" / "nav2.yaml").read_text(encoding="utf-8")
        )
        expected_footprint = [
            [0.165, 0.1825],
            [0.165, -0.1825],
            [-0.165, -0.1825],
            [-0.165, 0.1825],
        ]
        for costmap_name in ("local_costmap", "global_costmap"):
            with self.subTest(costmap=costmap_name):
                params = nav2_config[costmap_name][costmap_name]["ros__parameters"]
                self.assertEqual(
                    yaml.safe_load(params["footprint"]), expected_footprint
                )
                self.assertEqual(params["footprint_padding"], 0.015)

        monitor_config = yaml.safe_load(
            (BRINGUP / "config" / "collision_monitor.yaml").read_text(
                encoding="utf-8"
            )
        )["collision_monitor"]["ros__parameters"]
        self.assertEqual(monitor_config["cmd_vel_in_topic"], "/cmd_vel")
        self.assertEqual(monitor_config["cmd_vel_out_topic"], "/cmd_vel_safe")
        self.assertEqual(monitor_config["source_timeout"], 0.5)
        self.assertEqual(monitor_config["polygons"], ["FootprintApproach"])
        self.assertEqual(
            yaml.safe_load(monitor_config["FootprintApproach"]["points"]),
            [
                [0.18, 0.1975],
                [0.18, -0.1975],
                [-0.18, -0.1975],
                [-0.18, 0.1975],
            ],
        )
        self.assertEqual(monitor_config["observation_sources"], ["scan"])
        self.assertEqual(monitor_config["scan"]["topic"], "/scan")

        robot_launch = (BRINGUP / "launch" / "robot.launch.py").read_text(
            encoding="utf-8"
        )
        mapping_launch = (BRINGUP / "launch" / "mapping.launch.py").read_text(
            encoding="utf-8"
        )
        monitor_launch = (
            BRINGUP / "launch" / "collision_monitor.launch.py"
        ).read_text(encoding="utf-8")
        bridge = (
            ROOT
            / "ros2_ws"
            / "src"
            / "libbot_stm32"
            / "libbot_stm32"
            / "stm32_bridge.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"collision_monitor.launch.py"', robot_launch)
        self.assertIn('"cmd_vel_topic": "/cmd_vel_safe"', robot_launch)
        self.assertIn("'collision_monitor.launch.py'", mapping_launch)
        self.assertIn("'cmd_vel_topic': '/cmd_vel_safe'", mapping_launch)
        self.assertIn('package="nav2_collision_monitor"', monitor_launch)
        self.assertIn('"collision_monitor_params_file"', monitor_launch)
        self.assertNotIn('"params_file"', monitor_launch)
        self.assertIn('"autostart": True', monitor_launch)
        self.assertIn("declare_parameter('cmd_vel_topic', '/cmd_vel_safe')", bridge)

    def test_mecanum_description_and_simulation_command_path(self):
        description = ET.parse(
            ROOT
            / "ros2_ws"
            / "src"
            / "libot_description"
            / "urdf"
            / "libot_core.xacro"
        ).getroot()
        xacro_ns = {"xacro": "http://www.ros.org/wiki/xacro"}
        wheels = description.findall(".//xacro:mecanum_wheel", xacro_ns)
        self.assertEqual(len(wheels), 4)
        wheel_positions = {
            wheel.attrib["corner"]: (
                float(wheel.attrib["x"]),
                float(wheel.attrib["y"]),
            )
            for wheel in wheels
        }
        self.assertEqual(
            wheel_positions,
            {
                "front_left": (0.135, 0.165),
                "front_right": (0.135, -0.165),
                "rear_left": (-0.135, 0.165),
                "rear_right": (-0.135, -0.165),
            },
        )

        gazebo = (
            ROOT
            / "ros2_ws"
            / "src"
            / "libot_description"
            / "urdf"
            / "libot_gazebo.xacro"
        ).read_text(encoding="utf-8")
        self.assertIn("gz-sim-mecanum-drive-system", gazebo)
        self.assertIn("<wheelbase>0.27</wheelbase>", gazebo)
        self.assertIn("<wheel_separation>0.33</wheel_separation>", gazebo)
        self.assertIn("<wheel_radius>0.03</wheel_radius>", gazebo)
        self.assertNotIn("DiffDrive", gazebo)

        simulation_launch = (
            BRINGUP / "launch" / "simulation_navigation.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"collision_monitor.launch.py"', simulation_launch)
        self.assertIn('"use_sim_time": "true"', simulation_launch)
        bridge = yaml.safe_load(
            (BRINGUP / "config" / "ros_gz_bridge.yaml").read_text(encoding="utf-8")
        )
        command_bridge = next(
            entry for entry in bridge if entry["gz_topic_name"] == "/cmd_vel"
        )
        self.assertEqual(command_bridge["ros_topic_name"], "/cmd_vel_safe")
        self.assertEqual(
            command_bridge["direction"],
            "ROS_TO_GZ",
        )

        dependencies = (BRINGUP / "package.xml").read_text(encoding="utf-8")
        self.assertIn(
            "<exec_depend>nav2_mppi_controller</exec_depend>", dependencies
        )
        self.assertIn(
            "<exec_depend>libot_mppi_critics</exec_depend>", dependencies
        )

        compat = ROOT / "ros2_ws" / "src" / "libot_mppi_critics"
        compat_manifest = ET.parse(compat / "package.xml").getroot()
        self.assertEqual(compat_manifest.findtext("name"), "libot_mppi_critics")
        compat_build = (compat / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn(
            "mppi::critics::OmniConstraintCritic",
            (compat / "omni_constraint_critic.xml").read_text(encoding="utf-8"),
        )
        self.assertIn(
            "mppi::critics::MecanumCritic",
            (compat / "mecanum_critic.xml").read_text(encoding="utf-8"),
        )
        self.assertIn("if(COMPAT_SOURCES)", compat_build)

        run_instructions = (ROOT / "run.md").read_text(encoding="utf-8")
        mapping_instructions = (ROOT / "MAPPING_WORKFLOW.md").read_text(
            encoding="utf-8"
        )
        self.assertIn("src/libot_mppi_critics", run_instructions)
        self.assertIn("libot_mppi_critics", mapping_instructions)


if __name__ == "__main__":
    unittest.main()
