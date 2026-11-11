from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional
from urllib import request
from urllib.error import HTTPError, URLError


class RobotRuntimeGateway:
    """ROS-facing runtime adapter used by both Gazebo and the physical robot."""

    ZONE_POLICIES = {
        "KEEP_OUT": {
            "mask_layer": "keepout",
            "mask_value": "LETHAL",
            "blocked_actions": ("GOAL",),
        },
        "SPEED_LIMIT": {
            "mask_layer": "speed",
            "requires_max_speed": True,
            "mask_value": "SPEED_LIMIT",
        },
        "NO_STOP": {"blocked_actions": ("GOAL", "WAIT")},
    }
    ZONE_MASK_LAYERS = {
        "keepout": {
            "filename_suffix": "keepout_mask",
            "mode": "trinary",
            "launch_argument": "keepout_mask",
            "load_client": "_keepout_mask_load_client",
            "encoding": "LETHAL",
        },
        "speed": {
            "filename_suffix": "speed_mask",
            "mode": "scale",
            "launch_argument": "speed_mask",
            "load_client": "_speed_mask_load_client",
            "encoding": "SPEED_LIMIT",
        },
    }

    BATTERY_TIMEOUT_SEC = 3.0
    STM32_ACK_TIMEOUT_SEC = 0.15
    STM32_ERROR_STATUS_PREFIXES = ("UART_TX_ERROR,", "MCU_ERROR,")
    MANUAL_WHEEL_L_PLUS_W_M = 0.30
    NAV2_STARTUP_TIMEOUT_SEC = 60.0
    AUTO_NAVIGATION_RETRY_SEC = 5.0
    AMCL_LOCALIZATION_TIMEOUT_SEC = 20.0
    AMCL_MIN_STABLE_SAMPLES = 5
    AMCL_MAX_SCAN_AGE_SEC = 1.0
    AMCL_MAX_XY_VARIANCE = 0.04
    AMCL_MAX_YAW_VARIANCE = 0.03
    AMCL_MAX_SAMPLE_POSITION_SPREAD_M = 0.10
    AMCL_MAX_SAMPLE_YAW_SPREAD_RAD = 0.15
    NAVIGATION_PROGRESS_DISTANCE_M = 0.05
    FOLLOW_PATH_ERROR_NAMES = {
        100: "UNKNOWN",
        101: "INVALID_CONTROLLER",
        102: "TF_ERROR",
        103: "INVALID_PATH",
        104: "PATIENCE_EXCEEDED",
        105: "FAILED_TO_MAKE_PROGRESS",
        106: "NO_VALID_CONTROL",
        107: "CONTROLLER_TIMED_OUT",
    }
    NAV2_ACTIVATION_TIMEOUT_SEC = 30.0
    NAV_GOAL_SUBMISSION_TIMEOUT_SEC = 10.0
    NAVIGATION_NODES = (
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    )

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        config = settings or {}
        runtime = config.get("runtime", {})
        self.runtime_id = str(
            os.getenv("LIBOT_RUNTIME_ID", runtime.get("id", "real"))
        ).strip().lower()
        self.use_sim_time = self.runtime_id == "sim"
        self.simulation_world = str(os.getenv("LIBOT_SIM_WORLD", "library")).strip()
        self.simulation_speed = 1
        self.backend_url = os.getenv(
            "LIBOT_BACKEND_URL",
            runtime.get("backend_url", "http://127.0.0.1:5205"),
        ).rstrip("/")
        self.maps_dir = Path(os.getenv("LIBOT_MAPS_DIR", "/ros2_ws/maps")).resolve()
        self.telemetry_hz = max(1.0, min(float(runtime.get("telemetry_hz", 5.0)), 10.0))
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.twist = {"linearX": 0.0, "linearY": 0.0, "angularZ": 0.0}
        self.battery: Optional[float] = None
        self._last_battery_monotonic: Optional[float] = None
        self.localization_state = "UNLOCALIZED"
        self.navigation_mode_state = "STOPPED"
        self.navigation_state = "IDLE"
        self.active_goal_id: Optional[str] = None
        self.planned_path: list[Dict[str, float]] = []
        self.error: Optional[str] = None
        self._node: Any = None
        self._executor: Any = None
        self._action_client: Any = None
        self._goal_handle: Any = None
        self._active_goal_request: Optional[Dict[str, Any]] = None
        self._goal_timeout_timer: Optional[threading.Timer] = None
        self._goal_submission_timer: Optional[threading.Timer] = None
        self._goal_timeout_generation = 0
        self._goal_timeout_sec = 180.0
        self._goal_best_distance_remaining: Optional[float] = None
        self._goal_progress_pose: Optional[tuple[float, float]] = None
        self._initial_pose_pub: Any = None
        self._stop_pub: Any = None
        self._nomotion_update_client: Any = None
        self._navigation_lifecycle_client: Any = None
        self._navigation_lifecycle_started = False
        self._navigation_lifecycle_paused = False
        self._navigation_activation_lock = threading.Lock()
        self._tf_buffer: Any = None
        self._tf_listener: Any = None
        self._tf_timer: Any = None
        self._lock = threading.RLock()
        self._localization_condition = threading.Condition(self._lock)
        self._amcl_samples: deque[Dict[str, Any]] = deque(maxlen=self.AMCL_MIN_STABLE_SAMPLES)
        self._last_scan_monotonic: Optional[float] = None
        self._last_scan_ros_stamp_ns: Optional[int] = None
        self._localization_attempt_start_stamp_ns = 0
        self._localization_attempt_id = 0
        self._active_map_id: Optional[str] = None
        self._navigation_map_id: Optional[str] = None
        self._manual_command_lock = threading.Lock()
        self._stm32_ack_condition = threading.Condition()
        self._stm32_velocity_ack_sequence = 0
        self._stm32_status_sequence = 0
        self._stm32_last_status = ""
        self._running = False
        self._last_telemetry_error_log = 0.0
        self._last_map_error_log = 0.0
        self._map: Optional[Dict[str, Any]] = None
        self._last_map_post = 0.0
        self._manual_active = False
        self._last_manual_command = 0.0
        self._mapping_process: Optional[subprocess.Popen[Any]] = None
        self._navigation_process: Optional[subprocess.Popen[Any]] = None
        self._navigation_map_path: Optional[str] = None
        self._navigation_zones: list[Dict[str, Any]] = []
        self._navigation_speed_filter_multiplier = 0.01
        self._load_map_service_type: Any = None
        self._keepout_mask_load_client: Any = None
        self._speed_mask_load_client: Any = None
        self._active_map_id: Optional[str] = None
        self._pose_map_path: Optional[str] = None
        self.navigation_ready_state = False
        self.navigation_mode_state = "STOPPED"
        self._navigation_bootstrap_complete = False
        self._navigation_activation_attempted = False
        self._navigation_activation_lock = threading.Lock()
        self._manage_navigation_client: Any = None
        self._manage_lifecycle_type: Any = None
        self._get_lifecycle_state_type: Any = None
        self._lifecycle_state_clients: Dict[str, Any] = {}
        self._resume_navigation_after_manual_stop = False
        self._patrol_observation_condition = threading.Condition(self._lock)
        self._patrol_observation: Optional[Any] = None
        self._patrol_capture_active = False
        self._patrol_capture_observations: list[Any] = []
        self._patrol_capture_distance_m = 0.0
        self._patrol_capture_last_xy: Optional[tuple[float, float]] = None

    def start(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from nav2_msgs.action import NavigateToPose
            from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
            from nav_msgs.msg import OccupancyGrid, Odometry, Path
            from rclpy.action import ActionClient
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.parameter import Parameter
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import LaserScan
            from sensor_msgs.msg import BatteryState
            from std_msgs.msg import String
            from std_srvs.srv import Empty
            from tf2_ros import Buffer, TransformListener
            from lifecycle_msgs.srv import GetState
        except ImportError as exc:
            self.error = f"ROS_UNAVAILABLE: {exc}"
            return

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(
            f"libot_runtime_gateway_{self.runtime_id}",
            parameter_overrides=[
                Parameter("use_sim_time", value=self.use_sim_time)
            ],
        )
        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._stop_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._nomotion_update_client = self._node.create_client(
            Empty, "/request_nomotion_update"
        )
        self._navigation_lifecycle_client = self._node.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self._node.create_subscription(
            String, "/stm32_bridge/status", self._on_stm32_bridge_status, 10
        )
        # map_server publishes its map with transient-local durability.  A
        # volatile subscription can miss that one-shot publication when the
        # lifecycle manager activates map_server before DDS discovery has
        # matched this gateway.  Request transient-local durability so the
        # saved map is replayed and Nav2 startup does not time out waiting for
        # a map that was already published.
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._node.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos
        )
        self._node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self._node.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self._node.create_subscription(Path, "/plan", self._on_plan, 10)
        self._node.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self._node.create_subscription(String, "/libot/patrol_observation", self._on_patrol_observation, 10)
        self._tf_buffer = Buffer(node=self._node)
        self._tf_listener = TransformListener(self._tf_buffer, self._node)
        self._tf_timer = self._node.create_timer(0.1, self._update_pose_from_tf)
        self._manual_watchdog_timer = self._node.create_timer(0.05, self._manual_watchdog)
        self._action_client = ActionClient(self._node, NavigateToPose, "/navigate_to_pose")
        self._manage_navigation_client = self._node.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )
        self._manage_lifecycle_type = ManageLifecycleNodes
        self._get_lifecycle_state_type = GetState
        self._load_map_service_type = LoadMap
        self._keepout_mask_load_client = self._node.create_client(
            LoadMap, "/keepout_filter_mask_server/load_map"
        )
        self._speed_mask_load_client = self._node.create_client(
            LoadMap, "/speed_filter_mask_server/load_map"
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._running = True
        threading.Thread(target=self._executor.spin, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()
        threading.Thread(target=self._map_retry_loop, daemon=True).start()
        if self.runtime_id == "real":
            threading.Thread(target=self._auto_navigation_loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            self._navigation_timeout_clear_locked()
        self._publish_stop()
        self._terminate_process(self._mapping_process)
        self._terminate_process(self._navigation_process)
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        if self._node is not None:
            self._node.destroy_node()

    def wait_for_patrol_observation(self, timeout_sec: float) -> Optional[Any]:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._patrol_observation_condition:
            while self._patrol_observation is None and self._running:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._patrol_observation_condition.wait(min(remaining, 0.25))
            observation = self._patrol_observation
            self._patrol_observation = None
            return observation

    def begin_patrol_capture(self) -> None:
        with self._lock:
            self._patrol_capture_active = True
            self._patrol_capture_observations = []
            self._patrol_capture_distance_m = 0.0
            self._patrol_capture_last_xy = (float(self.pose["x"]), float(self.pose["y"]))

    def finish_patrol_capture(self) -> Dict[str, Any]:
        with self._lock:
            result = {
                "observations": list(self._patrol_capture_observations),
                "actual_distance_m": float(self._patrol_capture_distance_m),
            }
            self._patrol_capture_active = False
            self._patrol_capture_observations = []
            self._patrol_capture_last_xy = None
            return result

    def cancel_active_navigation(self) -> None:
        with self._lock:
            goal_handle = self._goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.error = f"Unable to cancel active navigation: {exc}"
        self._publish_stop()

    def navigation_ready(self, timeout_sec: float = 2.0) -> bool:
        if self._action_client is None or not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            return False
        with self._lock:
            ready = self.navigation_mode_state == "READY"
        if not ready:
            return False
        states = self._navigation_lifecycle_states(timeout_sec=timeout_sec)
        return all(state == 3 for state in states.values())

    def _on_patrol_observation(self, message: Any) -> None:
        try:
            observation = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            observation = {"type": "WARNING", "description": str(getattr(message, "data", ""))}
        with self._patrol_observation_condition:
            self._patrol_observation = observation
            if self._patrol_capture_active:
                self._patrol_capture_observations.append(observation)
            self._patrol_observation_condition.notify_all()

    def handle_request(self, payload: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(payload, dict):
            return None
        command = str(payload.get("command", "")).lower()
        if command == "get_simulation_speed":
            if not self.use_sim_time:
                return self._failure("Simulation speed is available only for the SIM runtime.")
            return {"status": "ok", "multiplier": self.simulation_speed}
        if command == "set_simulation_speed":
            return self._set_simulation_speed(payload)
        if command == "set_initial_pose":
            return self._set_initial_pose(payload.get("pose") or {})
        if command == "start_navigation":
            return self._start_navigation(payload)
        if command == "cancel_navigation":
            return self._cancel_navigation(str(payload.get("command_id", "")))
        if command == "manual_velocity":
            return self._manual_velocity(payload)
        if command == "manual_stop":
            if self._navigation_process is not None and self._navigation_process.poll() is None:
                return self._failure("Stop Nav2 before manual control.")
            with self._manual_command_lock:
                acked, ack_error = self._publish_stop_and_wait_for_stm32()
                hard_error = self._is_stm32_error_status(ack_error)
                with self._lock:
                    self._manual_active = False
                    resume_navigation = bool(payload.get("resume_navigation", False)) and self._resume_navigation_after_manual_stop
                    map_path = self._navigation_map_path
                    map_id = self._navigation_map_id
                    zones = list(self._navigation_zones)
                    if resume_navigation and not hard_error:
                        self._resume_navigation_after_manual_stop = False
                    elif resume_navigation:
                        self._resume_navigation_after_manual_stop = True
                if hard_error:
                    return self._failure(ack_error)
            if not resume_navigation:
                result = {
                    "status": "accepted",
                    "navigation_active": False,
                    "navigation_restarted": False,
                }
                if acked:
                    result["stm32_acknowledged"] = True
                return result
            if not map_path:
                with self._lock:
                    self._resume_navigation_after_manual_stop = True
                return self._failure("Nav2 cannot be restarted because no previously used map path is available.")
            result = self._start_navigation_mode(map_path, map_id, zones)
            if result.get("status") != "accepted":
                with self._lock:
                    self._resume_navigation_after_manual_stop = True
                return result
            result = {
                "status": "accepted",
                "navigation_active": True,
                "navigation_restarted": True,
            }
            if acked:
                result["stm32_acknowledged"] = True
            return result
        if command == "start_mapping":
            # The real runtime's auto-navigation worker may still be waiting for
            # a saved map. A mapping request is an explicit mode choice: keep
            # that worker from starting Nav2 while SLAM is active.
            self._navigation_bootstrap_complete = True
            return self._start_mapping()
        if command == "stop_mapping":
            self._terminate_process(self._mapping_process)
            self._mapping_process = None
            self._publish_stop()
            return {"status": "accepted"}
        if command == "save_map":
            return self._save_map(str(payload.get("file_stem", "")))
        if command == "start_navigation_mode":
            return self._start_navigation_mode(
                str(payload.get("map_path", "")),
                str(payload.get("map_id", "")).strip() or None,
                payload.get("zones") if isinstance(payload.get("zones"), list) else [],
            )
        if command == "update_navigation_zones":
            return self._update_navigation_zones(payload)
        if command == "stop_navigation_mode":
            self._terminate_process(self._navigation_process)
            self._navigation_process = None
            self._publish_stop()
            with self._lock:
                self._localization_attempt_id += 1
                self._amcl_samples.clear()
                self.localization_state = "UNLOCALIZED"
                self.navigation_mode_state = "STOPPED"
                self.navigation_state = "IDLE"
                self.active_goal_id = None
                self._goal_handle = None
                self._active_goal_request = None
                self._pose_map_path = None
                self._navigation_lifecycle_started = False
                self._navigation_lifecycle_paused = False
                self._active_map_id = None
                self._navigation_timeout_clear_locked()
                self.error = (
                    "INITIAL_POSE_REQUIRED: Set the robot's current pose on the map "
                    "before sending a navigation goal."
                )
                self._resume_navigation_after_manual_stop = bool(self._navigation_map_path)
            return {"status": "accepted", "navigation_active": False}
        return None

    def _set_simulation_speed(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not self.use_sim_time:
            return self._failure("Simulation speed is available only for the SIM runtime.")
        try:
            multiplier = int(payload.get("multiplier"))
        except (TypeError, ValueError, OverflowError):
            return self._failure("Simulation speed must be one of 1, 2, 3, or 4.")
        if multiplier not in {1, 2, 3, 4}:
            return self._failure("Simulation speed must be one of 1, 2, 3, or 4.")
        if not self.simulation_world or not re.fullmatch(r"[A-Za-z0-9_]+", self.simulation_world):
            return self._failure("The configured Gazebo world name is invalid.")

        request_text = f"real_time_factor: {float(multiplier):.1f}"
        try:
            result = subprocess.run(
                [
                    "gz", "service", "-s", f"/world/{self.simulation_world}/set_physics",
                    "--reqtype", "gz.msgs.Physics", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "2000", "--req", request_text,
                ],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return self._failure(f"Unable to change Gazebo simulation speed: {exc}")

        response_text = f"{result.stdout}\n{result.stderr}".strip()
        if result.returncode != 0 or not re.search(r"\bdata\s*:\s*true\b", response_text, re.IGNORECASE):
            detail = response_text or f"gz service exited with code {result.returncode}."
            return self._failure(f"Gazebo rejected simulation speed {multiplier}x: {detail}")
        self.simulation_speed = multiplier
        return {"status": "accepted", "multiplier": multiplier}

    def _manual_velocity(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if self._node is None or self._stop_pub is None:
            return self._failure(self.error or "ROS is unavailable.")
        if self._navigation_process is not None and self._navigation_process.poll() is None:
            return self._failure("Stop Nav2 before manual control.")
        try:
            values = (
                float(payload.get("linear_x", 0.0)),
                float(payload.get("linear_y", 0.0)),
                float(payload.get("angular_z", 0.0)),
            )
            if payload.get("max_wheel_speed_mm_s") is None:
                raise ValueError("Backend must provide max_wheel_speed_mm_s.")
            max_wheel_speed_mm_s = float(payload["max_wheel_speed_mm_s"])
            if not all(math.isfinite(value) for value in values):
                raise ValueError("Velocity must be finite.")
            if not math.isfinite(max_wheel_speed_mm_s) or max_wheel_speed_mm_s <= 0.0:
                raise ValueError("max_wheel_speed_mm_s must be a positive finite number.")
        except (TypeError, ValueError, OverflowError) as exc:
            return self._failure(str(exc))
        from geometry_msgs.msg import Twist

        message = Twist()
        message.linear.x = max(-0.7, min(0.7, values[0]))
        message.linear.y = max(-0.7, min(0.7, values[1]))
        message.angular.z = max(-0.5, min(0.5, values[2]))
        wheel_speeds = (
            message.linear.x + message.linear.y + self.MANUAL_WHEEL_L_PLUS_W_M * message.angular.z,
            message.linear.x - message.linear.y - self.MANUAL_WHEEL_L_PLUS_W_M * message.angular.z,
            message.linear.x - message.linear.y + self.MANUAL_WHEEL_L_PLUS_W_M * message.angular.z,
            message.linear.x + message.linear.y - self.MANUAL_WHEEL_L_PLUS_W_M * message.angular.z,
        )
        max_wheel_speed_mps = max_wheel_speed_mm_s / 1000.0
        peak_wheel_speed_mps = max(abs(speed) for speed in wheel_speeds)
        if peak_wheel_speed_mps > max_wheel_speed_mps:
            scale = max_wheel_speed_mps / peak_wheel_speed_mps
            message.linear.x *= scale
            message.linear.y *= scale
            message.angular.z *= scale
        with self._manual_command_lock:
            with self._lock:
                self._manual_active = True
                self._last_manual_command = time.monotonic()
            acked, ack_error = self._publish_velocity_and_wait_for_stm32(message)
            if not acked and self._is_stm32_error_status(ack_error):
                with self._lock:
                    self._manual_active = False
                self._publish_stop()
                return self._failure(ack_error)
        result = {
            "status": "accepted",
            "requested_velocity": {
                "linear_x": message.linear.x,
                "linear_y": message.linear.y,
                "angular_z": message.angular.z,
            },
            "max_wheel_speed_mm_s": max_wheel_speed_mm_s,
        }
        if acked:
            result["stm32_acknowledged"] = True
        return result

    def _on_stm32_bridge_status(self, message: Any) -> None:
        status = str(getattr(message, "data", "")).strip()
        with self._stm32_ack_condition:
            self._stm32_last_status = status
            self._stm32_status_sequence += 1
            if status == "ACK,VEL":
                self._stm32_velocity_ack_sequence += 1
            self._stm32_ack_condition.notify_all()

    def _publish_velocity_and_wait_for_stm32(self, message: Any) -> tuple[bool, str]:
        with self._stm32_ack_condition:
            ack_sequence = self._stm32_velocity_ack_sequence
            status_sequence = self._stm32_status_sequence
        self._stop_pub.publish(message)
        return self._wait_for_stm32_velocity_ack(ack_sequence, status_sequence)

    def _publish_stop_and_wait_for_stm32(self) -> tuple[bool, str]:
        with self._stm32_ack_condition:
            ack_sequence = self._stm32_velocity_ack_sequence
            status_sequence = self._stm32_status_sequence
        self._publish_stop()
        return self._wait_for_stm32_velocity_ack(ack_sequence, status_sequence)

    def _wait_for_stm32_velocity_ack(
        self,
        previous_ack_sequence: int,
        previous_status_sequence: int,
    ) -> tuple[bool, str]:
        with self._stm32_ack_condition:
            acknowledged = self._stm32_ack_condition.wait_for(
                lambda: (
                    self._stm32_velocity_ack_sequence > previous_ack_sequence
                    or (
                        self._stm32_status_sequence > previous_status_sequence
                        and self._is_stm32_error_status(self._stm32_last_status)
                    )
                ),
                timeout=self.STM32_ACK_TIMEOUT_SEC,
            )
            if self._stm32_velocity_ack_sequence > previous_ack_sequence:
                return True, ""
            if acknowledged and self._is_stm32_error_status(self._stm32_last_status):
                return False, self._stm32_last_status
            last_status = self._stm32_last_status or "no STM32 status received"
        warning = (
            "STM32 did not acknowledge the VEL command within "
            f"{self.STM32_ACK_TIMEOUT_SEC:.2f}s ({last_status}); command was still sent."
        )
        print(f"Warning: {warning}", flush=True)
        return False, warning

    @classmethod
    def _is_stm32_error_status(cls, status: str) -> bool:
        return str(status).startswith(cls.STM32_ERROR_STATUS_PREFIXES)

    def _manual_watchdog(self) -> None:
        with self._lock:
            expired = self._manual_active and time.monotonic() - self._last_manual_command > 0.3
            if expired:
                self._manual_active = False
        if expired:
            self._publish_stop()

    def _start_mapping(self) -> Dict[str, Any]:
        if self._node is None:
            return self._failure(self.error or "ROS is unavailable.")
        if self._navigation_process is not None and self._navigation_process.poll() is None:
            return self._failure("Stop Nav2 before starting SLAM mapping.")
        if self._mapping_process is not None and self._mapping_process.poll() is None:
            return {"status": "accepted", "message": "SLAM mapping is already running."}
        try:
            with self._lock:
                self._map = None
                self.localization_state = "UNLOCALIZED"
                self._pose_map_path = None
                self.error = None
            self._mapping_process = subprocess.Popen(
                [
                    "ros2", "launch", "libot_bringup", "mapping.launch.py",
                    "use_hardware:=false",
                    f"use_sim_time:={'true' if self.use_sim_time else 'false'}",
                ],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            time.sleep(0.25)
            if self._mapping_process.poll() is not None:
                self._mapping_process = None
                return self._failure("Could not start the SLAM mapping launch.")
            with self._lock:
                self._resume_navigation_after_manual_stop = False
            return {"status": "accepted"}
        except OSError as exc:
            return self._failure(f"Could not start SLAM mapping: {exc}")

    def _auto_navigation_loop(self) -> None:
        with self._lock:
            self.navigation_mode_state = "FETCHING_MAP"
        while self._running and not self._navigation_bootstrap_complete:
            # A user can start SLAM while this worker is waiting for the backend.
            # Do not race the explicit mapping mode by launching Nav2 afterward.
            if self._mapping_process is not None and self._mapping_process.poll() is None:
                self._navigation_bootstrap_complete = True
                return
            try:
                latest = self._get_backend_json(
                    f"{self.backend_url}/api/robot-runtimes/real/maps/latest"
                )
                if not isinstance(latest, dict) or not latest.get("id"):
                    # No saved map is the normal state before the first mapping
                    # session. Wait quietly instead of surfacing it as a runtime
                    # error in the mapping UI.
                    with self._lock:
                        self.navigation_mode_state = "FETCHING_MAP"
                        self.navigation_ready_state = False
                        self.error = None
                    deadline = time.monotonic() + self.AUTO_NAVIGATION_RETRY_SEC
                    while self._running and time.monotonic() < deadline:
                        time.sleep(0.1)
                    continue
                map_id = str(latest["id"])
                saved = self._get_backend_json(f"{self.backend_url}/api/maps/{map_id}")
                grid = self._grid_from_saved_map(saved)
                zones_response = self._get_backend_json(
                    f"{self.backend_url}/api/maps/{map_id}/zones"
                )
                zones = self._navigation_zones_from_backend(zones_response, map_id)
                yaml_path, _ = self._write_grid_files(
                    grid, self.maps_dir / f"backend_{map_id.replace('-', '')}"
                )
                result = self._start_navigation_mode(
                    str(yaml_path), map_id=map_id, zones=zones
                )
                if result.get("status") != "accepted":
                    raise RuntimeError(str(result.get("message") or "Could not start Nav2."))
                print(
                    "Auto-navigation loaded backend zones for map "
                    f"{map_id}: zones={result.get('zone_counts_by_type', {})}, "
                    f"mask_cells={result.get('mask_cell_counts_by_layer', {})}",
                    flush=True,
                )
                self._navigation_bootstrap_complete = True
                return
            except (
                OSError,
                ValueError,
                RuntimeError,
                HTTPError,
                URLError,
                json.JSONDecodeError,
            ) as exc:
                with self._lock:
                    self.navigation_mode_state = "FETCHING_MAP"
                    self.navigation_ready_state = False
                    self.error = f"AUTO_NAVIGATION_WAITING: {exc}"
                deadline = time.monotonic() + self.AUTO_NAVIGATION_RETRY_SEC
                while self._running and time.monotonic() < deadline:
                    time.sleep(0.1)

    @staticmethod
    def _get_backend_json(url: str) -> Any:
        req = request.Request(url, headers={"Accept": "application/json"})
        with request.urlopen(req, timeout=10.0) as response:
            body = response.read().decode("utf-8").strip()
            return json.loads(body) if body else None

    @staticmethod
    def _grid_from_saved_map(saved: Any) -> Dict[str, Any]:
        if not isinstance(saved, dict):
            raise ValueError("Saved map response is not an object.")
        data = saved.get("data")
        if not isinstance(data, dict) or data.get("format") != "occupancy-grid-v1":
            raise ValueError("Saved map is not an occupancy-grid-v1 map.")
        width = int(saved.get("width", 0))
        height = int(saved.get("height", 0))
        resolution = float(data.get("resolution", 0.0))
        origin = data.get("origin")
        cells = data.get("cells")
        if (
            width <= 0
            or height <= 0
            or not math.isfinite(resolution)
            or resolution <= 0
            or not isinstance(origin, list)
            or len(origin) < 3
            or not all(
                isinstance(value, (int, float)) and math.isfinite(float(value))
                for value in origin[:3]
            )
            or not isinstance(cells, list)
            or len(cells) != width * height
            or not all(isinstance(value, int) and -1 <= value <= 100 for value in cells)
        ):
            raise ValueError("Saved occupancy grid dimensions or values are invalid.")
        return {
            "timestamp": str(data.get("timestamp") or datetime.now(timezone.utc).isoformat()),
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin_x": float(origin[0]),
            "origin_y": float(origin[1]),
            "origin_yaw": float(origin[2]),
            "data": cells,
        }

    @staticmethod
    def _navigation_zones_from_backend(zones_response: Any, map_id: str) -> list[Dict[str, Any]]:
        if not isinstance(zones_response, list):
            raise ValueError("Backend map zones response is not a list.")

        normalized: list[Dict[str, Any]] = []
        for index, zone in enumerate(zones_response):
            if not isinstance(zone, dict):
                raise ValueError(f"Backend map zone {index} is not an object.")

            zone_map_id = zone.get("mapId", zone.get("map_id"))
            if str(zone_map_id or "").casefold() != map_id.casefold():
                raise ValueError(f"Backend map zone {index} belongs to a different map.")

            is_active = zone.get("isActive", zone.get("is_active"))
            if not isinstance(is_active, bool):
                raise ValueError(f"Backend map zone {index} is missing its active status.")
            if not is_active:
                continue

            normalized.append({
                "zone_type": zone.get("zoneType", zone.get("zone_type")),
                "min_x": zone.get("minX", zone.get("min_x")),
                "min_y": zone.get("minY", zone.get("min_y")),
                "max_x": zone.get("maxX", zone.get("max_x")),
                "max_y": zone.get("maxY", zone.get("max_y")),
                "width": zone.get("width"),
                "height": zone.get("height"),
                "rotation_angle": zone.get("rotationAngle", zone.get("rotation_angle", 0.0)),
                "max_speed": zone.get("maxSpeed", zone.get("max_speed", 0.0)),
            })

        return normalized

    @staticmethod
    def _write_grid_files(grid: Dict[str, Any], output: Path) -> tuple[Path, Path]:
        width = int(grid["width"])
        height = int(grid["height"])
        values = grid["data"]
        if width <= 0 or height <= 0 or len(values) != width * height:
            raise ValueError("Occupancy grid dimensions do not match its data.")
        output.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = output.with_suffix(".pgm")
        yaml_path = output.with_suffix(".yaml")
        pixels = bytearray()
        for row in range(height - 1, -1, -1):
            for value in values[row * width:(row + 1) * width]:
                occupancy = int(value)
                pixels.append(
                    205
                    if occupancy < 0
                    else max(0, min(254, round(254 - occupancy * 2.54)))
                )
        pgm_temp = pgm_path.with_suffix(".pgm.tmp")
        yaml_temp = yaml_path.with_suffix(".yaml.tmp")
        pgm_temp.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + pixels)
        yaml_temp.write_text(
            f"image: {pgm_path.name}\nmode: trinary\nresolution: {grid['resolution']}\n"
            f"origin: [{grid['origin_x']}, {grid['origin_y']}, {grid['origin_yaw']}]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
        pgm_temp.replace(pgm_path)
        yaml_temp.replace(yaml_path)
        return yaml_path, pgm_path

    @staticmethod
    def _controller_profile_for_runtime(runtime_id: str) -> str:
        return (
            "diff_drive"
            if str(runtime_id).strip().lower() == "sim"
            else "mecanum"
        )

    def _start_navigation_mode(
        self,
        map_path: str,
        map_id: Optional[str] = None,
        zones: Optional[list[Dict[str, Any]]] = None,
    ) -> Dict[str, Any]:
        if self._mapping_process is not None and self._mapping_process.poll() is None:
            return self._failure("Stop SLAM mapping before starting Nav2.")
        path = Path(map_path).resolve()
        if (
            not path.is_absolute()
            or path.suffix.lower() != ".yaml"
            or self.maps_dir not in path.parents
            or not path.is_file()
        ):
            return self._failure("The selected map YAML is not available on the robot.")
        if self._navigation_process is not None and self._navigation_process.poll() is None:
            with self._lock:
                previous_start_failed = self.navigation_mode_state == "ERROR"
            if not previous_start_failed:
                return self._failure("Nav2 is already running.")
            self._terminate_process(self._navigation_process)
            self._navigation_process = None
        try:
            try:
                normalized_zones = self._normalize_navigation_zones(zones or [])
                mask_paths, speed_multiplier, mask_cell_counts_by_layer = self._write_zone_masks(
                    path, normalized_zones
                )
            except (OSError, ValueError, RuntimeError, TypeError) as exc:
                return self._failure(f"Could not prepare navigation zone filters: {exc}")
            zone_counts_by_type = self._zone_counts_by_type(normalized_zones)
            required_mask_layers = self._required_zone_mask_layers(normalized_zones)
            empty_mask_layers = [
                layer for layer in required_mask_layers
                if mask_cell_counts_by_layer.get(layer, 0) == 0
            ]
            if empty_mask_layers:
                return self._failure(
                    "The selected map produced empty cells for required zone mask layer(s): "
                    + ", ".join(sorted(empty_mask_layers))
                    + ". Check that the zones overlap the selected map."
                )
            with self._lock:
                self._localization_attempt_id += 1
                self._amcl_samples.clear()
                self.localization_state = "UNLOCALIZED"
                self.navigation_mode_state = "FETCHING_MAP"
                self.navigation_state = "IDLE"
                self.active_goal_id = None
                self._goal_handle = None
                self._goal_timeout_generation += 1
                self._navigation_timeout_clear_locked()
                self._navigation_lifecycle_started = False
                self._navigation_lifecycle_paused = False
                self._navigation_map_id = map_id or path.stem
                self._active_map_id = self._navigation_map_id
                self._navigation_zones = normalized_zones
                self._navigation_speed_filter_multiplier = speed_multiplier
                self._active_goal_request = None
                self._pose_map_path = None
                self._map = None
                self._last_map_post = 0.0
                self.error = None
            controller_profile = self._controller_profile_for_runtime(
                self.runtime_id
            )
            defer_navigation_activation = not self.use_sim_time
            self._navigation_process = subprocess.Popen(
                [
                    "ros2", "launch", "libot_bringup", "navigation.launch.py",
                    f"map:={path}",
                    "use_hardware:=false",
                    "start_workflow_app:=false",
                    "use_rviz:=false",
                    # On the real robot, wait for AMCL's initial-pose TF
                    # before activating costmaps. Simulation keeps its
                    # standalone autostart behavior.
                    f"autostart_navigation:={'false' if defer_navigation_activation else 'true'}",
                    f"use_sim_time:={'true' if self.use_sim_time else 'false'}",
                    f"controller_profile:={controller_profile}",
                    "zones_enabled:=true",
                    *[
                        f"{config['launch_argument']}:={mask_paths[layer]}"
                        for layer, config in self.ZONE_MASK_LAYERS.items()
                    ],
                    f"speed_filter_multiplier:={speed_multiplier:.12g}",
                ],
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
            time.sleep(0.25)
            if self._navigation_process.poll() is not None:
                return_code = self._navigation_process.returncode
                self._navigation_process = None
                with self._lock:
                    self.navigation_mode_state = "ERROR"
                    self.error = f"NAV2_START_FAILED: launch exited with code {return_code}."
                return self._failure(
                    f"Nav2 launch exited immediately with code {return_code}; check the robot container logs."
                )

            startup_deadline = time.monotonic() + self.NAV2_STARTUP_TIMEOUT_SEC
            nav_lifecycle_ready = (
                self._navigation_lifecycle_client is not None
                and startup_deadline > time.monotonic()
                and self._navigation_lifecycle_client.wait_for_service(
                    timeout_sec=max(0.0, startup_deadline - time.monotonic())
                )
            )
            localization_ready = (
                self._nomotion_update_client is not None
                and startup_deadline > time.monotonic()
                and self._nomotion_update_client.wait_for_service(
                    timeout_sec=max(0.0, startup_deadline - time.monotonic())
                )
            )
            if not nav_lifecycle_ready or not localization_ready:
                self._terminate_process(self._navigation_process)
                self._navigation_process = None
                with self._lock:
                    self.navigation_mode_state = "ERROR"
                    self.error = "NAV2_START_FAILED: map, AMCL, or lifecycle services did not become ready."
                return self._failure(
                    "Map/AMCL or navigation lifecycle services did not become available within "
                    f"{self.NAV2_STARTUP_TIMEOUT_SEC:.0f} seconds; "
                    "the launch was stopped. Check the robot container logs."
                )

            filter_nodes = (
                "keepout_filter_mask_server",
                "keepout_costmap_filter_info_server",
                "speed_filter_mask_server",
                "speed_costmap_filter_info_server",
            )
            filters_ready = False
            while time.monotonic() < startup_deadline and self._navigation_process.poll() is None:
                filters_ready = all(self._lifecycle_state(name, 0.15) == 3 for name in filter_nodes)
                if filters_ready:
                    break
                time.sleep(0.2)
            if not filters_ready:
                self._terminate_process(self._navigation_process)
                self._navigation_process = None
                with self._lock:
                    self.navigation_mode_state = "ERROR"
                    self.error = "NAV2_START_FAILED: navigation zone filter servers did not become active."
                return self._failure("Nav2 zone filter servers did not become active before navigation startup timed out.")

            while time.monotonic() < startup_deadline:
                with self._lock:
                    map_received = self._map is not None
                if map_received:
                    break
                if self._navigation_process.poll() is not None:
                    break
                time.sleep(0.1)
            with self._lock:
                map_received = self._map is not None
            if not map_received:
                self._terminate_process(self._navigation_process)
                self._navigation_process = None
                with self._lock:
                    self.navigation_mode_state = "ERROR"
                    self.error = "NAV2_START_FAILED: map server did not publish a map."
                return self._failure("Nav2 map server did not publish /map before startup timed out.")

            remaining_startup = max(0.0, startup_deadline - time.monotonic())
            lifecycle_ready = (
                self._wait_for_navigation_nodes_unconfigured(timeout_sec=remaining_startup)
                if defer_navigation_activation
                else self._wait_for_navigation_nodes_active(timeout_sec=remaining_startup)
            )
            if not lifecycle_ready:
                states = self._navigation_lifecycle_states(timeout_sec=1.0)
                detail = self._format_lifecycle_states(states)
                error = f"NAV2_LIFECYCLE_NOT_READY: {detail}"
                print(error, flush=True)
                self._terminate_process(self._navigation_process)
                self._navigation_process = None
                with self._lock:
                    self.navigation_mode_state = "ERROR"
                    self._navigation_lifecycle_started = False
                    self._navigation_lifecycle_paused = False
                    self._navigation_map_path = None
                    self.error = error
                return self._failure(
                    "Nav2 launched, but its lifecycle servers did not reach the expected startup state: "
                    f"{detail}. The failed launch was stopped; check the robot container logs, then retry."
                )

            with self._lock:
                self._navigation_map_path = str(path)
                self._active_map_id = str(map_id) if map_id else None
                self._navigation_lifecycle_started = not defer_navigation_activation
                self._navigation_lifecycle_paused = False
                self.navigation_ready_state = self.use_sim_time
                self.navigation_mode_state = "READY" if self.use_sim_time else "WAITING_INITIAL_POSE"
                self._navigation_activation_attempted = False
                self._resume_navigation_after_manual_stop = False
                self.navigation_mode_state = "ACTIVATING" if self.use_sim_time else "WAITING_INITIAL_POSE"
                self.localization_state = "UNLOCALIZED"
                self._pose_map_path = None
                self.error = None if self.use_sim_time else (
                    "INITIAL_POSE_REQUIRED: Set the robot's current pose on the map before sending a goal."
                )

            if self.use_sim_time:
                if self._action_client is None or not self._action_client.wait_for_server(
                    timeout_sec=max(0.0, startup_deadline - time.monotonic())
                ):
                    self._terminate_process(self._navigation_process)
                    self._navigation_process = None
                    with self._lock:
                        self.navigation_mode_state = "ERROR"
                        self.error = "NAV2_ACTIVATION_FAILED: /navigate_to_pose is unavailable."
                    return self._failure("Nav2 action server did not become available after activation.")
                self._update_pose_from_tf()
                with self._lock:
                    self.navigation_mode_state = "READY"
            return {
                "status": "accepted",
                "navigation_mode_state": self.navigation_mode_state,
                "active_map_id": self._active_map_id,
                "zone_count": len(normalized_zones),
                "zone_counts_by_type": zone_counts_by_type,
                "mask_cell_counts_by_layer": mask_cell_counts_by_layer,
            }
        except OSError as exc:
            with self._lock:
                self.navigation_mode_state = "ERROR"
                self.error = f"NAV2_START_FAILED: {exc}"
            return self._failure(f"Could not start Nav2: {exc}")

    def _save_map(self, file_stem: str) -> Dict[str, Any]:
        with self._lock:
            grid = dict(self._map) if self._map is not None else None
        if grid is None:
            return self._failure("No live occupancy grid has been received.")
        output = Path(file_stem).resolve()
        if self.maps_dir not in output.parents:
            return self._failure("Map file path is invalid.")
        width = int(grid["width"])
        height = int(grid["height"])
        values = grid["data"]
        if width <= 0 or height <= 0 or len(values) != width * height:
            return self._failure("Occupancy grid dimensions do not match its data.")
        try:
            yaml_path, pgm_path = self._write_grid_files(grid, output)
            with self._lock:
                if self.localization_state == "LOCALIZED":
                    self._pose_map_path = str(yaml_path)
            return {
                "status": "accepted",
                "yaml_path": str(yaml_path),
                "pgm_path": str(pgm_path),
                "map": {
                    "timestamp": grid["timestamp"],
                    "width": width,
                    "height": height,
                    "resolution": grid["resolution"],
                    "originX": grid["origin_x"],
                    "originY": grid["origin_y"],
                    "originYaw": grid["origin_yaw"],
                    "data": values,
                },
            }
        except OSError as exc:
            return self._failure(f"Could not write the map files: {exc}")

    @staticmethod
    def _terminate_process(process: Optional[subprocess.Popen[Any]]) -> None:
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=2.0)

    def _set_initial_pose(self, pose: Dict[str, Any]) -> Dict[str, Any]:
        if self._node is None or self._initial_pose_pub is None:
            return self._failure(self.error or "ROS is unavailable.")
        values = self._pose_values(pose)
        if values is None:
            return self._failure("Initial pose requires finite x, y and yaw values.")
        if self.runtime_id == "sim":
            self._update_pose_from_tf()
            return {
                "status": "accepted",
                "localization_state": self.localization_state,
                "message": "SIM uses the Gazebo ground-truth pose.",
            }
        if (
            self._navigation_process is None
            or self._navigation_process.poll() is not None
            or not self._navigation_map_path
        ):
            return self._failure(
                "Load a map and start the navigation mode before setting the initial pose."
            )
        from geometry_msgs.msg import PoseWithCovarianceStamped

        x, y, yaw = values
        try:
            attempt_start_stamp_ns = int(self._node.get_clock().now().nanoseconds)
        except Exception:
            attempt_start_stamp_ns = 0
        with self._lock:
            if self.navigation_mode_state not in {
                "WAITING_INITIAL_POSE", "LOCALIZING", "ACTIVATING", "READY", "ERROR"
            }:
                return self._failure("Nav2 is not ready to accept an initial pose.")
            previous_goal_handle = self._goal_handle
            had_active_goal = (
                previous_goal_handle is not None
                or self.active_goal_id is not None
                or self.navigation_state in {"SENDING", "NAVIGATING", "CANCELING"}
            )
            self._goal_timeout_generation += 1
            self._navigation_timeout_clear_locked()
            self.active_goal_id = None
            self._goal_handle = None
            # Setting/retrying an initial pose is not itself a canceled
            # navigation. Only expose CANCELED when an actual goal was
            # interrupted; otherwise the navigation channel remains idle
            # while AMCL reports LOCALIZING independently.
            self.navigation_state = "CANCELED" if had_active_goal else "IDLE"
            self._localization_attempt_id += 1
            attempt_id = self._localization_attempt_id
            self._amcl_samples.clear()
            self._last_scan_monotonic = None
            self._last_scan_ros_stamp_ns = None
            self._localization_attempt_start_stamp_ns = attempt_start_stamp_ns
            self.localization_state = "LOCALIZING"
            self.navigation_mode_state = "LOCALIZING"
            self.error = None
            self._pose_map_path = self._navigation_map_path
            self._localization_condition.notify_all()

        if previous_goal_handle is not None:
            try:
                previous_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._publish_stop()
        with self._navigation_activation_lock:
            with self._lock:
                if attempt_id != self._localization_attempt_id:
                    return self._failure("This initial-pose request was superseded by a newer request.")
                lifecycle_started = self._navigation_lifecycle_started
                lifecycle_paused = self._navigation_lifecycle_paused
            if lifecycle_started and not lifecycle_paused:
                if not self._call_navigation_lifecycle("PAUSE", timeout_sec=5.0):
                    self._fail_localization_attempt(attempt_id, "NAV2_PAUSE_FAILED")
                    return self._failure("Nav2 could not pause before relocalization.")
                with self._lock:
                    self._navigation_lifecycle_paused = True

        with self._navigation_activation_lock:
            with self._lock:
                if attempt_id != self._localization_attempt_id:
                    return self._failure("This initial-pose request was superseded by a newer request.")
            for _ in range(3):
                message = PoseWithCovarianceStamped()
                message.header.frame_id = "map"
                message.header.stamp = self._node.get_clock().now().to_msg()
                message.pose.pose.position.x = x
                message.pose.pose.position.y = y
                message.pose.pose.orientation.z = math.sin(yaw / 2.0)
                message.pose.pose.orientation.w = math.cos(yaw / 2.0)
                message.pose.covariance[0] = 0.25
                message.pose.covariance[7] = 0.25
                message.pose.covariance[35] = 0.0685
                self._initial_pose_pub.publish(message)
                time.sleep(0.1)

        threading.Thread(
            target=self._localization_worker,
            args=(attempt_id,),
            daemon=True,
            name=f"libot-localization-{attempt_id}",
        ).start()
        return {"status": "accepted", "localization_state": "LOCALIZING"}

    def _localization_worker(self, attempt_id: int) -> None:
        if not self._running:
            return
        deadline = time.monotonic() + self.AMCL_LOCALIZATION_TIMEOUT_SEC
        next_nomotion_request = 0.0
        stable = False
        while self._running and time.monotonic() < deadline:
            with self._lock:
                if attempt_id != self._localization_attempt_id:
                    return

            now = time.monotonic()
            if now >= next_nomotion_request and self._nomotion_update_client is not None:
                try:
                    if self._nomotion_update_client.service_is_ready():
                        from std_srvs.srv import Empty

                        self._nomotion_update_client.call_async(Empty.Request())
                except Exception:
                    pass
                next_nomotion_request = now + 0.5

            tf_pose = self._lookup_map_pose()
            try:
                ros_now_ns = int(self._node.get_clock().now().nanoseconds)
            except Exception:
                ros_now_ns = 0
            with self._lock:
                samples = tuple(self._amcl_samples)
                scan_fresh = (
                    self._last_scan_monotonic is not None
                    and now - self._last_scan_monotonic <= self.AMCL_MAX_SCAN_AGE_SEC
                    and self._last_scan_ros_stamp_ns is not None
                    and ros_now_ns > 0
                    and -100_000_000 <= ros_now_ns - self._last_scan_ros_stamp_ns <= 1_000_000_000
                )
            if tf_pose is not None and scan_fresh and self._amcl_window_is_stable(samples):
                stable = True
                break
            with self._localization_condition:
                self._localization_condition.wait(timeout=0.1)

        if not self._running:
            return
        if not stable:
            self._fail_localization_attempt(attempt_id, "AMCL_NOT_STABLE")
            return

        with self._navigation_activation_lock:
            with self._lock:
                if attempt_id != self._localization_attempt_id:
                    return
                self.navigation_mode_state = "ACTIVATING"
                lifecycle_started = self._navigation_lifecycle_started

            command = "RESUME" if lifecycle_started and lifecycle_paused else None
            if not lifecycle_started:
                if self._get_lifecycle_state_type is None:
                    command = "STARTUP"
                else:
                    states = self._navigation_lifecycle_states(timeout_sec=1.0)
                    if all(state == 3 for state in states.values()):
                        with self._lock:
                            self._navigation_lifecycle_started = True
                            self._navigation_lifecycle_paused = False
                        lifecycle_started = True
                    elif any(state in {2, 3, 4} for state in states.values()):
                        self._fail_localization_attempt(
                            attempt_id,
                            "NAV2_PARTIAL_LIFECYCLE_STATE: "
                            + self._format_lifecycle_states(states),
                        )
                        return
                    elif all(state == 1 for state in states.values()):
                        command = "STARTUP"
                    else:
                        self._fail_localization_attempt(
                            attempt_id,
                            "NAV2_LIFECYCLE_STATE_UNAVAILABLE: "
                            + self._format_lifecycle_states(states),
                        )
                        return

            if command is not None and not self._call_navigation_lifecycle(
                command, timeout_sec=self.NAV2_ACTIVATION_TIMEOUT_SEC
            ):
                states = self._navigation_lifecycle_states(timeout_sec=1.0)
                detail = self._format_lifecycle_states(states)
                self._fail_localization_attempt(
                    attempt_id, f"NAV2_ACTIVATION_FAILED: {detail}"
                )
                return

            if not self._wait_for_navigation_nodes_active(timeout_sec=10.0):
                states = self._navigation_lifecycle_states(timeout_sec=1.0)
                detail = self._format_lifecycle_states(states)
                self._fail_localization_attempt(
                    attempt_id, f"NAV2_NODES_NOT_ACTIVE: {detail}"
                )
                return

            with self._lock:
                self._navigation_lifecycle_started = True
                self._navigation_lifecycle_paused = False
                still_current = attempt_id == self._localization_attempt_id
            if not still_current:
                paused = self._call_navigation_lifecycle("PAUSE", timeout_sec=5.0)
                if paused:
                    with self._lock:
                        self._navigation_lifecycle_paused = True
                return

            if self._action_client is None or not self._action_client.wait_for_server(
                timeout_sec=self.NAV2_ACTIVATION_TIMEOUT_SEC
            ):
                paused = self._call_navigation_lifecycle("PAUSE", timeout_sec=5.0)
                if paused:
                    with self._lock:
                        self._navigation_lifecycle_paused = True
                self._fail_localization_attempt(attempt_id, "NAV2_ACTIVATION_FAILED")
                return

            with self._lock:
                if attempt_id != self._localization_attempt_id:
                    stale_attempt = True
                else:
                    stale_attempt = False
            if stale_attempt:
                paused = self._call_navigation_lifecycle("PAUSE", timeout_sec=5.0)
                if paused:
                    with self._lock:
                        self._navigation_lifecycle_paused = True
                return
            with self._lock:
                self.localization_state = "LOCALIZED"
                self.navigation_mode_state = "READY"
                self.error = None
                self._pose_map_path = self._navigation_map_path

    def _call_navigation_lifecycle(self, command: str, timeout_sec: float) -> bool:
        client = self._navigation_lifecycle_client
        if client is None:
            return False
        try:
            deadline = time.monotonic() + max(0.0, timeout_sec)
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False
            from nav2_msgs.srv import ManageLifecycleNodes

            lifecycle_request = ManageLifecycleNodes.Request()
            command_value = getattr(type(lifecycle_request), command, None)
            if command_value is None:
                command_value = getattr(ManageLifecycleNodes.Request, command, None)
            if command_value is None:
                return False
            lifecycle_request.command = command_value
            future = client.call_async(lifecycle_request)
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(max(0.0, deadline - time.monotonic())):
                return False
            response = future.result()
            return bool(response and response.success)
        except Exception:
            return False

    def _lifecycle_state(self, node_name: str, timeout_sec: float = 1.0) -> Optional[int]:
        if self._node is None or self._get_lifecycle_state_type is None:
            return None
        try:
            client = self._lifecycle_state_clients.get(node_name)
            if client is None:
                client = self._node.create_client(
                    self._get_lifecycle_state_type, f"/{node_name}/get_state"
                )
                self._lifecycle_state_clients[node_name] = client
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return None
            future = client.call_async(self._get_lifecycle_state_type.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(max(0.0, timeout_sec)):
                return None
            response = future.result()
            return int(response.current_state.id) if response is not None else None
        except Exception:
            return None

    def _navigation_lifecycle_states(self, timeout_sec: float = 1.0) -> Dict[str, Optional[int]]:
        states: Dict[str, Optional[int]] = dict.fromkeys(self.NAVIGATION_NODES)
        if self._node is None or self._get_lifecycle_state_type is None:
            return states

        pending = {}
        for name in self.NAVIGATION_NODES:
            try:
                client = self._lifecycle_state_clients.get(name)
                if client is None:
                    client = self._node.create_client(
                        self._get_lifecycle_state_type, f"/{name}/get_state"
                    )
                    self._lifecycle_state_clients[name] = client
                if client.service_is_ready():
                    pending[name] = client.call_async(
                        self._get_lifecycle_state_type.Request()
                    )
            except Exception:
                continue

        deadline = time.monotonic() + max(0.0, timeout_sec)
        while pending and time.monotonic() < deadline:
            completed = [name for name, future in pending.items() if future.done()]
            for name in completed:
                future = pending.pop(name)
                try:
                    response = future.result()
                    if response is not None:
                        states[name] = int(response.current_state.id)
                except Exception:
                    pass
            if pending:
                time.sleep(0.01)
        return states

    @staticmethod
    def _format_lifecycle_states(states: Dict[str, Optional[int]]) -> str:
        labels = {1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized"}
        return ", ".join(
            f"{name}={labels.get(state, 'unavailable')}"
            for name, state in states.items()
        )

    def _wait_for_navigation_nodes_active(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while self._running and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            states = self._navigation_lifecycle_states(
                timeout_sec=min(1.0, remaining)
            )
            if all(state == 3 for state in states.values()):
                return True
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        return False

    def _wait_for_navigation_nodes_unconfigured(self, timeout_sec: float) -> bool:
        """Wait for Nav2 lifecycle services before initial-pose localization."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while self._running and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            states = self._navigation_lifecycle_states(
                timeout_sec=min(1.0, remaining)
            )
            if all(state == 1 for state in states.values()):
                return True
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        return False

    def _fail_localization_attempt(self, attempt_id: int, error_code: str) -> None:
        with self._lock:
            if attempt_id != self._localization_attempt_id:
                return
            self.localization_state = "UNLOCALIZED"
            self.navigation_mode_state = "ERROR"
            self.error = error_code
            self._amcl_samples.clear()
        print(error_code, flush=True)
        self._publish_stop()

    @classmethod
    def _amcl_window_is_stable(cls, samples: Any) -> bool:
        if len(samples) < cls.AMCL_MIN_STABLE_SAMPLES:
            return False
        window = samples[-cls.AMCL_MIN_STABLE_SAMPLES:]
        if any(
            not all(math.isfinite(float(sample[key])) for key in ("x", "y", "yaw", "cov_x", "cov_y", "cov_yaw"))
            or sample["cov_x"] < 0.0
            or sample["cov_y"] < 0.0
            or sample["cov_yaw"] < 0.0
            or sample["cov_x"] > cls.AMCL_MAX_XY_VARIANCE
            or sample["cov_y"] > cls.AMCL_MAX_XY_VARIANCE
            or sample["cov_yaw"] > cls.AMCL_MAX_YAW_VARIANCE
            for sample in window
        ):
            return False
        max_position_spread = max(
            math.hypot(first["x"] - second["x"], first["y"] - second["y"])
            for index, first in enumerate(window)
            for second in window[index + 1:]
        )
        max_yaw_spread = max(
            abs(math.atan2(
                math.sin(first["yaw"] - second["yaw"]),
                math.cos(first["yaw"] - second["yaw"]),
            ))
            for index, first in enumerate(window)
            for second in window[index + 1:]
        )
        return (
            max_position_spread <= cls.AMCL_MAX_SAMPLE_POSITION_SPREAD_M
            and max_yaw_spread <= cls.AMCL_MAX_SAMPLE_YAW_SPREAD_RAD
        )

    def _lookup_map_pose(self) -> Optional[Dict[str, float]]:
        if self._tf_buffer is None:
            return None
        try:
            from rclpy.time import Time

            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            ).transform
        except Exception:
            return None
        rotation = transform.rotation
        return {
            "x": float(transform.translation.x),
            "y": float(transform.translation.y),
            "yaw": math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            ),
        }

    def _normalize_navigation_zones(self, zones: list[Dict[str, Any]]) -> list[Dict[str, Any]]:
        normalized: list[Dict[str, Any]] = []
        for index, zone in enumerate(zones):
            if not isinstance(zone, dict):
                raise ValueError(f"Navigation zone {index} must be an object.")
            zone_type = str(zone.get("zone_type", "")).strip().upper()
            policy = self.ZONE_POLICIES.get(zone_type)
            if policy is None:
                raise ValueError(f"Unsupported navigation zone type '{zone_type}'.")
            bounds = {key: float(zone.get(key)) for key in ("min_x", "min_y", "max_x", "max_y")}
            if not all(math.isfinite(value) for value in bounds.values()):
                raise ValueError("Navigation zone bounds must be finite.")
            if bounds["min_x"] >= bounds["max_x"] or bounds["min_y"] >= bounds["max_y"]:
                raise ValueError("Navigation zone bounds must have positive width and height.")
            width_value = zone.get("width")
            height_value = zone.get("height")
            if (width_value is None) != (height_value is None):
                raise ValueError("Oriented navigation zones must provide both width and height.")
            width = height = None
            if width_value is not None:
                width, height = float(width_value), float(height_value)
                if not math.isfinite(width) or not math.isfinite(height) or width <= 0.0 or height <= 0.0:
                    raise ValueError("Oriented navigation zone dimensions must be positive and finite.")
            try:
                rotation_angle = float(zone.get("rotation_angle", 0.0) or 0.0)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError("Navigation zone rotation must be numeric.") from exc
            if not math.isfinite(rotation_angle):
                raise ValueError("Navigation zone rotation must be finite.")
            max_speed = 0.0
            if policy.get("requires_max_speed"):
                try:
                    max_speed = float(zone.get("max_speed", 0.0) or 0.0)
                except (TypeError, ValueError, OverflowError) as exc:
                    raise ValueError("Navigation zone speed must be numeric.") from exc
                if not math.isfinite(max_speed) or max_speed <= 0.0:
                    raise ValueError("SPEED_LIMIT requires max_speed greater than zero.")
            normalized.append({
                "zone_type": zone_type,
                **bounds,
                "width": width,
                "height": height,
                "rotation_angle": rotation_angle,
                "max_speed": max_speed,
            })
        return normalized

    @staticmethod
    def _zone_contains_world(zone: Dict[str, Any], world_x: float, world_y: float) -> bool:
        width, height = zone.get("width"), zone.get("height")
        if width is None or height is None:
            return zone["min_x"] <= world_x <= zone["max_x"] and zone["min_y"] <= world_y <= zone["max_y"]

        center_x = (zone["min_x"] + zone["max_x"]) / 2.0
        center_y = (zone["min_y"] + zone["max_y"]) / 2.0
        dx, dy = world_x - center_x, world_y - center_y
        angle = zone["rotation_angle"]
        cosine, sine = math.cos(angle), math.sin(angle)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        return abs(local_x) <= width / 2.0 and abs(local_y) <= height / 2.0

    @staticmethod
    def _zone_world_corners(zone: Dict[str, Any]) -> list[tuple[float, float]]:
        width, height = zone.get("width"), zone.get("height")
        center_x = (zone["min_x"] + zone["max_x"]) / 2.0
        center_y = (zone["min_y"] + zone["max_y"]) / 2.0
        if width is None or height is None:
            return [
                (zone["min_x"], zone["min_y"]), (zone["min_x"], zone["max_y"]),
                (zone["max_x"], zone["min_y"]), (zone["max_x"], zone["max_y"]),
            ]

        cosine, sine = math.cos(zone["rotation_angle"]), math.sin(zone["rotation_angle"])
        return [
            (center_x + cosine * local_x - sine * local_y,
             center_y + sine * local_x + cosine * local_y)
            for local_x, local_y in (
                (-width / 2.0, -height / 2.0), (-width / 2.0, height / 2.0),
                (width / 2.0, -height / 2.0), (width / 2.0, height / 2.0),
            )
        ]

    @staticmethod
    def _zone_counts_by_type(zones: list[Dict[str, Any]]) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for zone in zones:
            zone_type = zone["zone_type"]
            counts[zone_type] = counts.get(zone_type, 0) + 1
        return counts

    @classmethod
    def _required_zone_mask_layers(cls, zones: list[Dict[str, Any]]) -> set[str]:
        return {
            policy["mask_layer"]
            for zone in zones
            if (policy := cls.ZONE_POLICIES.get(zone["zone_type"])) is not None
            and policy.get("mask_layer")
        }

    def _update_navigation_zones(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        map_id = str(payload.get("map_id", "")).strip()
        zones = payload.get("zones")
        with self._lock:
            map_path = self._navigation_map_path
            active_map_id = self._navigation_map_id
            previous_zones = list(self._navigation_zones)
            previous_multiplier = self._navigation_speed_filter_multiplier

        if not map_path or self._navigation_process is None or self._navigation_process.poll() is not None:
            return self._failure("Nav2 is not running, so navigation zones cannot be updated.")
        if not map_id or map_id.casefold() != str(active_map_id or "").casefold():
            return self._failure("The requested map is not the map currently loaded in Nav2.")
        if not isinstance(zones, list):
            return self._failure("Navigation zones must be provided as a list.")

        try:
            normalized_zones = self._normalize_navigation_zones(zones)
            mask_paths, _, mask_cell_counts_by_layer = self._write_zone_masks(
                Path(map_path), normalized_zones, previous_multiplier
            )
        except (OSError, ValueError, RuntimeError, TypeError) as exc:
            return self._failure(f"Could not prepare navigation zone filters: {exc}")
        required_mask_layers = self._required_zone_mask_layers(normalized_zones)
        empty_mask_layers = [
            layer for layer in required_mask_layers
            if mask_cell_counts_by_layer.get(layer, 0) == 0
        ]
        if empty_mask_layers:
            return self._failure(
                "The selected map produced empty cells for required zone mask layer(s): "
                + ", ".join(sorted(empty_mask_layers))
                + ". Check that the zones overlap the selected map."
            )

        mask_result = self._load_navigation_zone_masks(mask_paths)
        if mask_result is not None:
            self._restore_navigation_zone_filters(previous_zones, previous_multiplier, map_path)
            return self._failure(mask_result)

        with self._lock:
            self._navigation_zones = normalized_zones
        replan_result = self._replan_active_navigation_goal()
        return {
            "status": "accepted",
            "map_id": map_id,
            "zone_count": len(normalized_zones),
            "zone_counts_by_type": self._zone_counts_by_type(normalized_zones),
            "mask_cell_counts_by_layer": mask_cell_counts_by_layer,
            "navigation_replanned": replan_result is True,
        }

    def _replan_active_navigation_goal(self) -> Optional[bool]:
        with self._lock:
            if self.navigation_state not in {"SENDING", "NAVIGATING"}:
                return None
            goal_request = dict(self._active_goal_request or {})
            if not goal_request:
                return False
            previous_goal_handle = self._goal_handle
            self._goal_timeout_generation += 1
            self._navigation_timeout_clear_locked()
            self._goal_submission_timeout_clear_locked()
            self.active_goal_id = None
            self._goal_handle = None
            self._active_goal_request = None
            self.navigation_state = "IDLE"

        self._publish_stop()
        if previous_goal_handle is not None:
            try:
                cancel_future = previous_goal_handle.cancel_goal_async()
                deadline = time.monotonic() + 3.0
                while not cancel_future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
            except Exception as exc:
                print(f"Could not cancel the previous goal before replanning: {exc}", flush=True)

        response = self._start_navigation(goal_request)
        if response.get("status") != "accepted":
            with self._lock:
                self.navigation_state = "FAILED"
                self.error = str(response.get("message") or "The active goal could not be replanned after zone changes.")
            return False
        return True

    def _load_mask_map(self, client: Any, path: Path) -> Optional[str]:
        if client is None or self._load_map_service_type is None:
            return "Nav2 navigation zone map services are unavailable."
        if not client.wait_for_service(timeout_sec=5.0):
            return "Nav2 navigation zone map service did not become available."
        request_message = self._load_map_service_type.Request()
        request_message.map_url = str(path)
        future = client.call_async(request_message)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return "Nav2 timed out while loading a navigation zone map."
        try:
            response = future.result()
        except Exception as exc:
            return f"Nav2 failed to load a navigation zone map: {exc}"
        if response is None or int(response.result) != 0:
            result_code = "unknown" if response is None else str(response.result)
            return f"Nav2 rejected a navigation zone map (result {result_code})."
        return None

    def _load_navigation_zone_masks(self, mask_paths: Dict[str, Path]) -> Optional[str]:
        for layer, config in self.ZONE_MASK_LAYERS.items():
            client = getattr(self, config["load_client"])
            result = self._load_mask_map(client, mask_paths[layer])
            if result is not None:
                return result
        return None

    def _restore_navigation_zone_filters(
        self,
        zones: list[Dict[str, Any]],
        multiplier: float,
        map_path: str,
    ) -> None:
        try:
            mask_paths, _, _ = self._write_zone_masks(
                Path(map_path), zones, multiplier
            )
            self._load_navigation_zone_masks(mask_paths)
        except (OSError, ValueError, RuntimeError, TypeError) as exc:
            print(f"Could not restore previous navigation zone filters: {exc}", flush=True)

    def navigation_zone_blocking_action(
        self,
        action: str,
        pose: Optional[Dict[str, Any]] = None,
    ) -> Optional[str]:
        requested_action = str(action).strip().upper()
        candidate = pose if isinstance(pose, dict) else self.pose
        try:
            x, y = float(candidate["x"]), float(candidate["y"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        with self._lock:
            zones = list(self._navigation_zones)
        for zone in zones:
            policy = self.ZONE_POLICIES.get(zone["zone_type"], {})
            if (
                requested_action in policy.get("blocked_actions", ())
                and self._zone_contains_world(zone, x, y)
            ):
                return zone["zone_type"]
        return None

    @staticmethod
    def _pgm_dimensions(image_path: Path) -> tuple[int, int]:
        data = image_path.read_bytes()
        index = 0

        def token() -> bytes:
            nonlocal index
            while index < len(data):
                if data[index] in b" \t\r\n\v\f":
                    index += 1
                elif data[index] == ord("#"):
                    while index < len(data) and data[index] not in b"\r\n":
                        index += 1
                else:
                    break
            start = index
            while index < len(data) and data[index] not in b" \t\r\n\v\f#":
                index += 1
            if start == index:
                raise ValueError("The saved map image has an invalid PGM header.")
            return data[start:index]

        if token() not in (b"P2", b"P5"):
            raise ValueError("Saved navigation maps must use a PGM image.")
        width, height = int(token()), int(token())
        max_value = int(token())
        if width <= 0 or height <= 0 or max_value <= 0:
            raise ValueError("The saved map PGM dimensions are invalid.")
        return width, height

    def _write_zone_masks(
        self,
        map_yaml_path: Path,
        zones: list[Dict[str, Any]],
        speed_multiplier_override: Optional[float] = None,
    ) -> tuple[Dict[str, Path], float, Dict[str, int]]:
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML is required to load saved map metadata for zone navigation.") from exc

        metadata = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
        if not isinstance(metadata, dict) or not isinstance(metadata.get("image"), str):
            raise ValueError("Saved map YAML must define an image file.")
        image_path = (map_yaml_path.parent / metadata["image"]).resolve()
        if self.maps_dir not in image_path.parents or not image_path.is_file():
            raise ValueError("The saved map image is not available in the robot maps directory.")
        width, height = self._pgm_dimensions(image_path)
        resolution = float(metadata.get("resolution", 0.0))
        origin = metadata.get("origin")
        if not math.isfinite(resolution) or resolution <= 0.0 or not isinstance(origin, list) or len(origin) < 3:
            raise ValueError("Saved map YAML has invalid resolution or origin metadata.")
        origin_x, origin_y, origin_yaw = (float(value) for value in origin[:3])
        if not all(math.isfinite(value) for value in (origin_x, origin_y, origin_yaw)):
            raise ValueError("Saved map origin must contain finite coordinates.")

        layer_cells = {
            layer: bytearray(width * height)
            for layer in self.ZONE_MASK_LAYERS
        }
        speed_zones = [
            zone for zone in zones
            if self.ZONE_POLICIES[zone["zone_type"]].get("mask_value") == "SPEED_LIMIT"
        ]
        max_zone_speed = max((zone["max_speed"] for zone in speed_zones), default=1.0)
        speed_multiplier = (
            float(speed_multiplier_override)
            if speed_multiplier_override is not None
            else max_zone_speed / 100.0
        )
        if not math.isfinite(speed_multiplier) or speed_multiplier <= 0.0:
            raise ValueError("Speed filter multiplier must be a positive finite value.")
        cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)

        for zone in zones:
            policy = self.ZONE_POLICIES[zone["zone_type"]]
            layer = policy.get("mask_layer")
            if layer is None:
                continue
            layer_config = self.ZONE_MASK_LAYERS.get(layer)
            if layer_config is None:
                raise ValueError(f"Navigation zone layer '{layer}' is not configured.")
            cells = layer_cells[layer]
            encoding = policy.get("mask_value")
            if encoding == "LETHAL":
                cell_value = 1
            elif encoding == "SPEED_LIMIT":
                cell_value = max(1, min(100, round(zone["max_speed"] / speed_multiplier)))
            else:
                raise ValueError(f"Navigation zone type '{zone['zone_type']}' has no supported mask encoding.")
            corners = self._zone_world_corners(zone)
            local = []
            for world_x, world_y in corners:
                dx, dy = world_x - origin_x, world_y - origin_y
                local.append((cosine * dx + sine * dy, -sine * dx + cosine * dy))
            min_col = max(0, int(math.floor(min(point[0] for point in local) / resolution)))
            max_col = min(width - 1, int(math.floor(max(point[0] for point in local) / resolution)))
            min_row = max(0, height - 1 - int(math.floor(max(point[1] for point in local) / resolution)))
            max_row = min(height - 1, height - 1 - int(math.floor(min(point[1] for point in local) / resolution)))
            if max_col < min_col or max_row < min_row:
                continue
            for row in range(min_row, max_row + 1):
                local_y = (height - row - 0.5) * resolution
                for col in range(min_col, max_col + 1):
                    local_x = (col + 0.5) * resolution
                    world_x = origin_x + cosine * local_x - sine * local_y
                    world_y = origin_y + sine * local_x + cosine * local_y
                    if self._zone_contains_world(zone, world_x, world_y):
                        offset = row * width + col
                        self._merge_zone_mask_cell(cells, offset, cell_value, layer_config)

        # Keep sub-cell zones represented by at least their center cell on the raster.
        for zone in zones:
            policy = self.ZONE_POLICIES[zone["zone_type"]]
            layer = policy.get("mask_layer")
            if layer is None:
                continue
            layer_config = self.ZONE_MASK_LAYERS[layer]
            encoding = policy.get("mask_value")
            if encoding == "LETHAL":
                cell_value = 1
            elif encoding == "SPEED_LIMIT":
                cell_value = max(1, min(100, round(zone["max_speed"] / speed_multiplier)))
            else:
                continue
            world_x = (zone["min_x"] + zone["max_x"]) / 2.0
            world_y = (zone["min_y"] + zone["max_y"]) / 2.0
            dx, dy = world_x - origin_x, world_y - origin_y
            local_x = cosine * dx + sine * dy
            local_y = -sine * dx + cosine * dy
            col = int(math.floor(local_x / resolution))
            grid_row = int(math.floor(local_y / resolution))
            image_row = height - 1 - grid_row
            if 0 <= col < width and 0 <= image_row < height:
                offset = image_row * width + col
                self._merge_zone_mask_cell(layer_cells[layer], offset, cell_value, layer_config)

        stem = re.sub(r"[^A-Za-z0-9_-]", "_", map_yaml_path.stem)
        mask_paths: Dict[str, Path] = {}
        mask_cell_counts_by_layer: Dict[str, int] = {}

        def write_mask(yaml_path: Path, cells: bytearray, mode: str, encoding: str) -> None:
            pgm_path = yaml_path.with_suffix(".pgm")
            with pgm_path.open("w", encoding="ascii", newline="\n") as image:
                image.write(f"P2\n{width} {height}\n255\n")
                for row in range(height):
                    values = []
                    for col in range(width):
                        value = cells[row * width + col]
                        if encoding == "LETHAL":
                            values.append("0" if value else "255")
                        elif encoding == "SPEED_LIMIT":
                            values.append(str(round((100 - value) * 255 / 100)))
                        else:
                            raise ValueError(f"Unsupported navigation mask encoding '{encoding}'.")
                    image.write(" ".join(values) + "\n")
            yaml_path.write_text(
                f"image: {pgm_path.name}\nresolution: {resolution:.12g}\n"
                f"origin: [{origin_x:.12g}, {origin_y:.12g}, {origin_yaw:.12g}]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: {mode}\n",
                encoding="utf-8",
            )

        for layer, config in self.ZONE_MASK_LAYERS.items():
            yaml_path = self.maps_dir / f"{stem}_{config['filename_suffix']}.yaml"
            cells = layer_cells[layer]
            write_mask(yaml_path, cells, config["mode"], config["encoding"])
            mask_paths[layer] = yaml_path
            mask_cell_counts_by_layer[layer] = sum(value != 0 for value in cells)
        return mask_paths, speed_multiplier, mask_cell_counts_by_layer

    @staticmethod
    def _merge_zone_mask_cell(cells: bytearray, offset: int, value: int, config: Dict[str, Any]) -> None:
        if config["encoding"] == "LETHAL":
            cells[offset] = max(cells[offset], value)
        elif config["encoding"] == "SPEED_LIMIT":
            cells[offset] = value if cells[offset] == 0 else min(cells[offset], value)
        else:
            raise ValueError(f"Unsupported navigation mask encoding '{config['encoding']}'.")

    def _start_navigation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        target = payload.get("target") or {}
        values = self._pose_values(target)
        if values is None:
            return self._failure("Navigation target requires finite x, y and theta values.")
        blocking_zone = self.navigation_zone_blocking_action("GOAL", {"x": values[0], "y": values[1]})
        if blocking_zone is not None:
            return self._failure(f"Navigation target is inside a {blocking_zone} zone.")
        if self._node is None or self._action_client is None:
            return self._failure(self.error or "ROS is unavailable.")
        command_id = str(payload.get("command_id") or "").strip()
        if not command_id:
            return self._failure("Navigation goal requires a command_id.")
        if self.use_sim_time and self._node.get_clock().now().nanoseconds <= 0:
            return self._failure(
                "SIM_CLOCK_UNAVAILABLE: No nonzero ROS simulated time has been received on /clock."
            )
        try:
            timeout_sec = max(1.0, min(float(payload.get("timeout_sec", 180)), 600.0))
        except (TypeError, ValueError, OverflowError):
            timeout_sec = 180.0
        with self._lock:
            if self.navigation_state in {"SENDING", "NAVIGATING", "CANCELING"}:
                return self._failure("A navigation goal is already active.")
            if self.navigation_mode_state != "READY" or self.localization_state != "LOCALIZED":
                return self._failure("Nav2 is not READY and localized.")
            self._navigation_timeout_clear_locked()
            self._goal_timeout_generation += 1
            generation = self._goal_timeout_generation
            self.active_goal_id = command_id
            self._active_goal_request = dict(payload)
            self.navigation_state = "SENDING"
            self.error = None
            self._goal_timeout_sec = timeout_sec
            self._goal_best_distance_remaining = None
            self._goal_progress_pose = (self.pose["x"], self.pose["y"])
            self._start_goal_submission_timeout_locked(command_id, generation)

        if not self._action_client.server_is_ready():
            self._action_client.wait_for_server(timeout_sec=2.0)
        if not self._action_client.server_is_ready():
            with self._lock:
                if self.active_goal_id == command_id and self._goal_timeout_generation == generation:
                    self._navigation_timeout_clear_locked()
                    self.navigation_state = "FAILED"
                    self.error = "NAV2_UNAVAILABLE"
                    self.active_goal_id = None
            return self._failure("Nav2 action server is unavailable.")

        from nav2_msgs.action import NavigateToPose

        x, y, yaw = values
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        try:
            future = self._action_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: self._on_feedback(
                    feedback, command_id, generation
                ),
            )
            future.add_done_callback(
                lambda result: self._on_goal_response(result, command_id, generation)
            )
        except Exception as exc:
            with self._lock:
                if self.active_goal_id == command_id and self._goal_timeout_generation == generation:
                    self._navigation_timeout_clear_locked()
                    self.navigation_state = "FAILED"
                    self.error = str(exc)
                    self.active_goal_id = None
            return self._failure(f"Unable to send navigation goal: {exc}")
        return {"status": "accepted", "command_id": command_id}

    def _cancel_navigation(self, command_id: str) -> Dict[str, Any]:
        with self._lock:
            if command_id and self.active_goal_id and command_id != self.active_goal_id:
                return self._failure("The requested command is not the active goal.")
            goal_handle = self._goal_handle
            goal_pending = self.navigation_state == "SENDING" and goal_handle is None
            self.navigation_state = "CANCELING"
            if goal_pending:
                self._navigation_progress_timeout_clear_locked()
            else:
                self._navigation_timeout_clear_locked()
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._publish_stop()
        return {"status": "accepted", "command_id": self.active_goal_id}

    def _on_goal_response(self, future: Any, command_id: str, generation: int) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                with self._lock:
                    if self.active_goal_id != command_id or self._goal_timeout_generation != generation:
                        return
                    self._navigation_timeout_clear_locked()
                    self.navigation_state = "REJECTED"
                    self.error = "GOAL_REJECTED"
                    self._goal_handle = None
                    self.active_goal_id = None
                    self._active_goal_request = None
                return
            with self._lock:
                if self.active_goal_id != command_id or self._goal_timeout_generation != generation:
                    stale = True
                else:
                    stale = False
                    cancel_requested = self.navigation_state == "CANCELING"
                    self._goal_submission_timeout_clear_locked()
                    self._goal_handle = goal_handle
                    if not cancel_requested:
                        self.navigation_state = "NAVIGATING"
                        self._reset_goal_timeout_locked(command_id, generation)
            if stale:
                goal_handle.cancel_goal_async()
                return
            if cancel_requested:
                goal_handle.cancel_goal_async()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._on_navigation_result(result, command_id, generation)
            )
        except Exception as exc:
            with self._lock:
                if self.active_goal_id == command_id and self._goal_timeout_generation == generation:
                    self._navigation_timeout_clear_locked()
                    self.navigation_state = "FAILED"
                    self.error = str(exc)
                    self._goal_handle = None
                    self.active_goal_id = None
                    self._active_goal_request = None

    def _on_navigation_result(self, future: Any, command_id: str, generation: int) -> None:
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
            state = {4: "SUCCEEDED", 5: "CANCELED", 6: "FAILED"}.get(status, f"STATUS_{status}")
        except Exception as exc:
            state = "FAILED"
            result_error = str(exc)
        else:
            action_result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(action_result, "error_code", 0) or 0)
            error_message = str(getattr(action_result, "error_msg", "") or "").strip()
            if state == "FAILED":
                details = [f"code={error_code}"]
                controller_error_name = self.FOLLOW_PATH_ERROR_NAMES.get(error_code)
                if controller_error_name:
                    details.append(controller_error_name)
                if error_message:
                    details.append(error_message)
                result_error = f"NAVIGATION_FAILED: {'; '.join(details)}"
            elif state.startswith("STATUS_"):
                result_error = f"NAVIGATION_RESULT_{state}"
            else:
                result_error = None
        with self._lock:
            if self.active_goal_id != command_id or self._goal_timeout_generation != generation:
                return
            self._navigation_timeout_clear_locked()
            self.navigation_state = state
            self._goal_handle = None
            self.error = result_error
            if state in {"SUCCEEDED", "CANCELED", "FAILED"}:
                self.active_goal_id = None
                self._active_goal_request = None

    def _on_goal_timeout(self, command_id: Optional[str], generation: int) -> None:
        with self._lock:
            if (
                not command_id
                or self.active_goal_id != command_id
                or self._goal_timeout_generation != generation
            ):
                return
            goal_handle = self._goal_handle
            self.navigation_state = "FAILED"
            self.error = "NO_PROGRESS_TIMEOUT"
            self.active_goal_id = None
            self._goal_handle = None
            self._active_goal_request = None
            self._goal_timeout_generation += 1
            self._navigation_timeout_clear_locked()
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._publish_stop()

    def _on_goal_submission_timeout(self, command_id: str, generation: int) -> None:
        with self._lock:
            if (
                self.active_goal_id != command_id
                or self._goal_timeout_generation != generation
                or self.navigation_state not in {"SENDING", "CANCELING"}
            ):
                return
            self.navigation_state = "FAILED"
            self.error = "GOAL_SUBMISSION_TIMEOUT"
            self.active_goal_id = None
            self._goal_handle = None
            self._active_goal_request = None
            self._goal_timeout_generation += 1
            self._navigation_timeout_clear_locked()
        self._publish_stop()

    def _on_feedback(self, feedback: Any, command_id: str, generation: int) -> None:
        with self._lock:
            if self.active_goal_id != command_id or self._goal_timeout_generation != generation:
                return
            self.navigation_state = "NAVIGATING"
            detail = getattr(feedback, "feedback", feedback)
            distance = getattr(detail, "distance_remaining", None)
            try:
                distance = float(distance)
                if not math.isfinite(distance):
                    distance = None
            except (TypeError, ValueError, OverflowError):
                distance = None

            current_pose = (float(self.pose["x"]), float(self.pose["y"]))
            moved = False
            if self._goal_progress_pose is None:
                self._goal_progress_pose = current_pose
            else:
                moved = math.hypot(
                    current_pose[0] - self._goal_progress_pose[0],
                    current_pose[1] - self._goal_progress_pose[1],
                ) >= self.NAVIGATION_PROGRESS_DISTANCE_M
            distance_progress = (
                distance is not None
                and self._goal_best_distance_remaining is not None
                and self._goal_best_distance_remaining - distance >= self.NAVIGATION_PROGRESS_DISTANCE_M
            )
            if self._goal_best_distance_remaining is None and distance is not None:
                self._goal_best_distance_remaining = distance
            elif distance_progress:
                self._goal_best_distance_remaining = distance
            if moved or distance_progress:
                self._goal_progress_pose = current_pose
                self._reset_goal_timeout_locked(command_id, generation)

    def _navigation_timeout_clear_locked(self) -> None:
        self._goal_submission_timeout_clear_locked()
        self._navigation_progress_timeout_clear_locked()

    def _goal_submission_timeout_clear_locked(self) -> None:
        timer = self._goal_submission_timer
        self._goal_submission_timer = None
        if timer is not None:
            timer.cancel()

    def _navigation_progress_timeout_clear_locked(self) -> None:
        timer = self._goal_timeout_timer
        self._goal_timeout_timer = None
        if timer is not None:
            timer.cancel()
        self._goal_best_distance_remaining = None
        self._goal_progress_pose = None

    def _start_goal_submission_timeout_locked(self, command_id: str, generation: int) -> None:
        self._goal_submission_timeout_clear_locked()
        timer = threading.Timer(
            self.NAV_GOAL_SUBMISSION_TIMEOUT_SEC,
            self._on_goal_submission_timeout,
            args=(command_id, generation),
        )
        timer.daemon = True
        self._goal_submission_timer = timer
        timer.start()

    def _reset_goal_timeout_locked(self, command_id: str, generation: int) -> None:
        timer = self._goal_timeout_timer
        if timer is not None:
            timer.cancel()
        timer = threading.Timer(
            self._goal_timeout_sec,
            self._on_goal_timeout,
            args=(command_id, generation),
        )
        timer.daemon = True
        self._goal_timeout_timer = timer
        timer.start()

    def _on_amcl_pose(self, message: Any) -> None:
        if self.runtime_id == "sim":
            return
        pose = message.pose.pose
        with self._lock:
            self.pose = self._message_pose(pose)
            if self._navigation_process is not None and self._navigation_process.poll() is None:
                self._pose_map_path = self._navigation_map_path
            if self.navigation_mode_state == "LOCALIZING":
                stamp = getattr(getattr(message, "header", None), "stamp", None)
                stamp_ns = (
                    int(getattr(stamp, "sec", 0)) * 1_000_000_000
                    + int(getattr(stamp, "nanosec", 0))
                    if stamp is not None else 0
                )
                if (
                    self._localization_attempt_start_stamp_ns > 0
                    and stamp_ns < self._localization_attempt_start_stamp_ns
                ):
                    return
                covariance = message.pose.covariance
                self._amcl_samples.append({
                    "x": self.pose["x"],
                    "y": self.pose["y"],
                    "yaw": self.pose["yaw"],
                    "cov_x": float(covariance[0]),
                    "cov_y": float(covariance[7]),
                    "cov_yaw": float(covariance[35]),
                    "received_at": time.monotonic(),
                })
                self._localization_condition.notify_all()

    def _on_scan(self, message: Any) -> None:
        with self._lock:
            self._last_scan_monotonic = time.monotonic()
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            self._last_scan_ros_stamp_ns = (
                int(getattr(stamp, "sec", 0)) * 1_000_000_000
                + int(getattr(stamp, "nanosec", 0))
                if stamp is not None else None
            )
            self._localization_condition.notify_all()

    def _update_pose_from_tf(self) -> None:
        pose = self._lookup_map_pose()
        if pose is None:
            return
        with self._lock:
            self.pose = pose
            self._record_patrol_pose_locked(pose)
            if self.runtime_id == "sim":
                self.localization_state = "LOCALIZED"
            if (
                self.runtime_id == "sim"
                and self._navigation_process is not None
                and self._navigation_process.poll() is None
            ):
                self._pose_map_path = self._navigation_map_path
            elif self._mapping_process is not None and self._mapping_process.poll() is None:
                self._pose_map_path = None

    def _on_odom(self, message: Any) -> None:
        twist = message.twist.twist
        with self._lock:
            self.twist = {
                "linearX": float(twist.linear.x),
                "linearY": float(twist.linear.y),
                "angularZ": float(twist.angular.z),
            }

    def _record_patrol_pose_locked(self, pose: Dict[str, Any]) -> None:
        if not self._patrol_capture_active:
            return
        try:
            current = (float(pose["x"]), float(pose["y"]))
        except (KeyError, TypeError, ValueError):
            return
        previous = self._patrol_capture_last_xy
        if previous is not None:
            distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
            if math.isfinite(distance):
                self._patrol_capture_distance_m += distance
        self._patrol_capture_last_xy = current

    def _on_map(self, message: Any) -> None:
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        grid = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "origin_x": float(message.info.origin.position.x),
            "origin_y": float(message.info.origin.position.y),
            "origin_yaw": yaw,
            "data": [int(value) for value in message.data],
        }
        now = time.monotonic()
        with self._lock:
            self._map = grid
            should_post = (
                self.runtime_id == "real"
                and now - self._last_map_post >= 1.0
            )
            if should_post:
                self._last_map_post = now
        if should_post:
            try:
                self._post_map(grid)
            except Exception as exc:
                if now - self._last_map_error_log >= 5.0:
                    print(f"Map delivery failed: {exc}", flush=True)
                    self._last_map_error_log = now

    def _post_map(self, grid: Dict[str, Any]) -> None:
        payload = {
            "timestamp": grid["timestamp"],
            "width": grid["width"],
            "height": grid["height"],
            "resolution": grid["resolution"],
            "originX": grid["origin_x"],
            "originY": grid["origin_y"],
            "originYaw": grid["origin_yaw"],
            "data": grid["data"],
        }
        req = request.Request(
            f"{self.backend_url}/api/robot-runtimes/{self.runtime_id}/map",
            data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with request.urlopen(req, timeout=10.0) as response:
                response.read()
        except HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")[:500]
            raise RuntimeError(f"BE rejected occupancy map (HTTP {exc.code}): {body}") from exc

    def _on_plan(self, message: Any) -> None:
        points = [{"x": float(p.pose.position.x), "y": float(p.pose.position.y)} for p in message.poses]
        with self._lock:
            self.planned_path = points[:500]

    def _on_battery(self, message: Any) -> None:
        percentage = float(message.percentage)
        if 0.0 <= percentage <= 1.0:
            percentage *= 100.0

        with self._lock:
            if (not math.isfinite(percentage)
                    or not 0.0 <= percentage <= 100.0
                    or getattr(message, "present", True) is False):
                self.battery = None
                self._last_battery_monotonic = None
                return

            self.battery = percentage
            self._last_battery_monotonic = time.monotonic()

    def _battery_for_telemetry(self, now: Optional[float] = None) -> Optional[float]:
        if self.battery is None or self._last_battery_monotonic is None:
            return None
        current = time.monotonic() if now is None else now
        if current - self._last_battery_monotonic > self.BATTERY_TIMEOUT_SEC:
            return None
        return self.battery

    def _telemetry_loop(self) -> None:
        interval = 1.0 / self.telemetry_hz
        while self._running:
            started = time.monotonic()
            try:
                self._post_telemetry()
            except Exception as exc:
                now = time.monotonic()
                if now - self._last_telemetry_error_log >= 5.0:
                    print(f"Telemetry delivery failed: {exc}", flush=True)
                    self._last_telemetry_error_log = now
            time.sleep(max(0.0, interval - (time.monotonic() - started)))

    def _map_retry_loop(self) -> None:
        while self._running:
            self._retry_map_delivery()
            time.sleep(1.0)

    def _retry_map_delivery(self) -> None:
        if self.runtime_id != "real":
            return
        now = time.monotonic()
        with self._lock:
            if self._map is None or now - self._last_map_post < 5.0:
                return
            grid = dict(self._map)
            self._last_map_post = now
        try:
            self._post_map(grid)
        except Exception as exc:
            if now - self._last_map_error_log >= 5.0:
                print(f"Map delivery retry failed: {exc}", flush=True)
                self._last_map_error_log = now

    def _post_telemetry(self) -> None:
        with self._lock:
            mapping_active = self._mapping_process is not None and self._mapping_process.poll() is None
            navigation_active = (
                self._navigation_process is not None
                and self._navigation_process.poll() is None
                and self._navigation_map_path is not None
            )
            if self.use_sim_time and self._action_client is not None:
                self.navigation_ready_state = self._action_client.server_is_ready()
                self.navigation_mode_state = "READY" if self.navigation_ready_state else "FETCHING_MAP"
            if not navigation_active and self.navigation_ready_state:
                if not self.use_sim_time:
                    self.navigation_ready_state = False
                    self.navigation_mode_state = "ERROR"
                    self.error = "NAV2_PROCESS_EXITED: The navigation launch process is no longer running."
            payload = {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "pose": dict(self.pose),
                "twist": dict(self.twist),
                "battery": self._battery_for_telemetry(),
                "localizationState": self.localization_state,
                "navigationState": self.navigation_state,
                "activeGoalId": self.active_goal_id,
                "plannedPath": list(self.planned_path),
                "error": self.error,
                "mappingActive": mapping_active,
                "navigationActive": navigation_active,
                "navigationReady": bool(
                    self.navigation_mode_state == "READY"
                    and self._action_client is not None
                    and self._action_client.server_is_ready()
                ),
                "navigationModeState": self.navigation_mode_state,
                "activeMapId": self._active_map_id,
            }
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(
            f"{self.backend_url}/api/robot-runtimes/{self.runtime_id}/telemetry",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with request.urlopen(req, timeout=2.0):
            pass

    def _publish_stop(self) -> None:
        if self._stop_pub is None:
            return
        from geometry_msgs.msg import Twist

        stop_message = Twist()
        for _ in range(3):
            self._stop_pub.publish(stop_message)

    @staticmethod
    def _pose_values(pose: Dict[str, Any]) -> Optional[tuple[float, float, float]]:
        try:
            values = (float(pose["x"]), float(pose["y"]), float(pose.get("yaw", pose.get("theta", 0.0))))
            return values if all(math.isfinite(value) for value in values) else None
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _message_pose(pose: Any) -> Dict[str, float]:
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return {"x": float(pose.position.x), "y": float(pose.position.y), "yaw": yaw}

    @staticmethod
    def _failure(message: str) -> Dict[str, Any]:
        return {"status": "error", "message": message}
