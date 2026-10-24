from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional
from urllib import request
from urllib.error import HTTPError, URLError


class RobotRuntimeGateway:
    """ROS-facing runtime adapter used by both Gazebo and the physical robot."""

    BATTERY_TIMEOUT_SEC = 3.0
    STM32_ACK_TIMEOUT_SEC = 0.15
    STM32_ERROR_STATUS_PREFIXES = ("UART_TX_ERROR,", "MCU_ERROR,")
    MANUAL_WHEEL_L_PLUS_W_M = 0.30
    NAV2_STARTUP_TIMEOUT_SEC = 60.0
    AUTO_NAVIGATION_RETRY_SEC = 5.0
    INITIAL_POSE_TF_TIMEOUT_SEC = 15.0
    GOAL_RESPONSE_TIMEOUT_SEC = 3.0
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
        self.navigation_state = "IDLE"
        self.active_goal_id: Optional[str] = None
        self.planned_path: list[Dict[str, float]] = []
        self.error: Optional[str] = None
        self._node: Any = None
        self._executor: Any = None
        self._action_client: Any = None
        self._goal_handle: Any = None
        self._goal_timeout_timer: Optional[threading.Timer] = None
        self._initial_pose_pub: Any = None
        self._stop_pub: Any = None
        self._tf_buffer: Any = None
        self._tf_listener: Any = None
        self._tf_timer: Any = None
        self._lock = threading.RLock()
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

    def start(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from nav2_msgs.action import NavigateToPose
            from nav2_msgs.srv import ManageLifecycleNodes
            from nav_msgs.msg import OccupancyGrid, Odometry, Path
            from rclpy.action import ActionClient
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.parameter import Parameter
            from sensor_msgs.msg import BatteryState
            from std_msgs.msg import String
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
        self._node.create_subscription(
            String, "/stm32_bridge/status", self._on_stm32_bridge_status, 10
        )
        self._node.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        self._node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 20)
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
        if self.use_sim_time:
            return bool(
                self._action_client is not None
                and self._action_client.wait_for_server(timeout_sec=timeout_sec)
            )
        with self._lock:
            ready = self.navigation_ready_state
        return bool(
            ready
            and self._action_client is not None
            and self._action_client.wait_for_server(timeout_sec=timeout_sec)
        )

    def _on_patrol_observation(self, message: Any) -> None:
        try:
            observation = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            observation = {"type": "WARNING", "description": str(getattr(message, "data", ""))}
        with self._patrol_observation_condition:
            self._patrol_observation = observation
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
                    map_id = self._active_map_id
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
            result = self._start_navigation_mode(map_path, map_id=map_id)
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
                map_id=str(payload.get("map_id") or "") or None,
            )
        if command == "stop_navigation_mode":
            self._terminate_process(self._navigation_process)
            self._navigation_process = None
            self._publish_stop()
            with self._lock:
                self.localization_state = "UNLOCALIZED"
                self._pose_map_path = None
                self.navigation_ready_state = False
                self.navigation_mode_state = "STOPPED"
                self._active_map_id = None
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
            try:
                latest = self._get_backend_json(
                    f"{self.backend_url}/api/robot-runtimes/real/maps/latest"
                )
                if not isinstance(latest, dict) or not latest.get("id"):
                    raise RuntimeError("Backend does not have a saved robot map yet.")
                map_id = str(latest["id"])
                saved = self._get_backend_json(
                    f"{self.backend_url}/api/maps/{map_id}"
                )
                grid = self._grid_from_saved_map(saved)
                yaml_path, _ = self._write_grid_files(
                    grid, self.maps_dir / f"backend_{map_id.replace('-', '')}"
                )
                result = self._start_navigation_mode(str(yaml_path), map_id=map_id)
                if result.get("status") != "accepted":
                    raise RuntimeError(str(result.get("message") or "Could not start Nav2."))
                self._navigation_bootstrap_complete = True
                return
            except (OSError, ValueError, RuntimeError, HTTPError, URLError, json.JSONDecodeError) as exc:
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
            return json.loads(response.read().decode("utf-8"))

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
            width <= 0 or height <= 0
            or not math.isfinite(resolution) or resolution <= 0
            or not isinstance(origin, list) or len(origin) < 3
            or not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in origin[:3])
            or not isinstance(cells, list) or len(cells) != width * height
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
                pixels.append(205 if occupancy < 0 else max(0, min(254, round(254 - occupancy * 2.54))))
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

    def _start_navigation_mode(self, map_path: str, map_id: Optional[str] = None) -> Dict[str, Any]:
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
            return self._failure("Nav2 is already running.")
        try:
            with self._lock:
                initial_pose = (
                    dict(self.pose)
                    if self.localization_state == "LOCALIZED"
                    and self._pose_map_path == str(path)
                    else None
                )
            self._navigation_process = subprocess.Popen(
                [
                    "ros2", "launch", "libot_bringup", "navigation.launch.py",
                    f"map:={path}",
                    "use_hardware:=false",
                    "start_workflow_app:=false",
                    "use_rviz:=false",
                    f"autostart_navigation:={'true' if self.use_sim_time else 'false'}",
                    f"use_sim_time:={'true' if self.use_sim_time else 'false'}",
                ],
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
            time.sleep(0.25)
            if self._navigation_process.poll() is not None:
                return_code = self._navigation_process.returncode
                self._navigation_process = None
                return self._failure(
                    f"Nav2 launch exited immediately with code {return_code}; check the robot container logs."
                )
            stack_ready = (
                self._action_client is not None
                and self._action_client.wait_for_server(timeout_sec=self.NAV2_STARTUP_TIMEOUT_SEC)
                if self.use_sim_time
                else self._wait_for_lifecycle_nodes(("map_server", "amcl"), self.NAV2_STARTUP_TIMEOUT_SEC)
            )
            if not stack_ready:
                self._terminate_process(self._navigation_process)
                self._navigation_process = None
                return self._failure(
                    "Nav2 localization stack did not become active within "
                    f"{self.NAV2_STARTUP_TIMEOUT_SEC:.0f} seconds; "
                    "the launch was stopped. Check the robot container logs."
                )
            with self._lock:
                self._navigation_map_path = str(path)
                self._active_map_id = str(map_id) if map_id else None
                self.navigation_ready_state = self.use_sim_time
                self.navigation_mode_state = "READY" if self.use_sim_time else "WAITING_INITIAL_POSE"
                self._navigation_activation_attempted = False
                self._resume_navigation_after_manual_stop = False
                if initial_pose is None:
                    self.localization_state = "UNLOCALIZED"
                    self._pose_map_path = None
                    self.error = (
                        "INITIAL_POSE_REQUIRED: Set the robot's current pose on the map "
                        "before sending a navigation goal."
                    )
                else:
                    self.error = None
            if initial_pose is not None and self.use_sim_time:
                threading.Thread(
                    target=self._seed_nav2_pose,
                    args=(initial_pose,),
                    daemon=True,
                ).start()
            return {
                "status": "accepted",
                "active_map_id": self._active_map_id,
                "navigation_ready": self.navigation_ready_state,
                "navigation_mode_state": self.navigation_mode_state,
            }
        except OSError as exc:
            return self._failure(f"Could not start Nav2: {exc}")

    def _seed_nav2_pose(self, pose: Dict[str, float]) -> None:
        for delay in (4.0, 1.0, 1.0):
            if not self._running:
                return
            time.sleep(delay)
            if self._navigation_process is None or self._navigation_process.poll() is not None:
                return
            if self._node is None or self._initial_pose_pub is None:
                return
            from geometry_msgs.msg import PoseWithCovarianceStamped

            message = PoseWithCovarianceStamped()
            message.header.frame_id = "map"
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.pose.pose.position.x = pose["x"]
            message.pose.pose.position.y = pose["y"]
            message.pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
            message.pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
            message.pose.covariance[0] = 0.1
            message.pose.covariance[7] = 0.1
            message.pose.covariance[35] = 0.05
            self._initial_pose_pub.publish(message)

    @staticmethod
    def _wait_for_future(future: Any, timeout_sec: float) -> bool:
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        return completed.wait(timeout=max(0.0, timeout_sec))

    def _lifecycle_state(self, node_name: str, timeout_sec: float = 2.0) -> Optional[int]:
        if self._node is None:
            return None
        client = self._lifecycle_state_clients.get(node_name)
        if client is None:
            client = self._node.create_client(
                self._get_lifecycle_state_type, f"/{node_name}/get_state"
            )
            self._lifecycle_state_clients[node_name] = client
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(self._get_lifecycle_state_type.Request())
        if not self._wait_for_future(future, timeout_sec):
            return None
        try:
            return int(future.result().current_state.id)
        except Exception:
            return None

    def _wait_for_lifecycle_nodes(self, node_names: tuple[str, ...], timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while self._running and time.monotonic() < deadline:
            if all(self._lifecycle_state(name, 0.5) == 3 for name in node_names):
                return True
            time.sleep(0.2)
        return False

    def _navigation_lifecycle_summary(self) -> str:
        labels = {1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized"}
        states = []
        for name in self.NAVIGATION_NODES:
            state = self._lifecycle_state(name, 1.0)
            states.append(f"{name}={labels.get(state, 'unavailable')}")
        return ", ".join(states)

    def _manage_navigation_lifecycle(self, command: int, timeout_sec: float) -> bool:
        client = self._manage_navigation_client
        if client is None or not client.wait_for_service(timeout_sec=min(timeout_sec, 5.0)):
            return False
        request_message = self._manage_lifecycle_type.Request()
        request_message.command = command
        future = client.call_async(request_message)
        if not self._wait_for_future(future, timeout_sec):
            return False
        try:
            return bool(future.result().success)
        except Exception:
            return False

    def _has_map_transform(self) -> bool:
        if self._tf_buffer is None:
            return False
        try:
            from rclpy.time import Time

            self._tf_buffer.lookup_transform("map", "base_footprint", Time())
            return True
        except Exception:
            return False

    def _activate_navigation_after_initial_pose(self, pose: Dict[str, float]) -> None:
        if not self._navigation_activation_lock.acquire(blocking=False):
            return
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped

            for delay in (0.0, 0.5, 1.0):
                if delay:
                    time.sleep(delay)
                if not self._running or self._initial_pose_pub is None or self._node is None:
                    return
                message = PoseWithCovarianceStamped()
                message.header.frame_id = "map"
                message.header.stamp = self._node.get_clock().now().to_msg()
                message.pose.pose.position.x = pose["x"]
                message.pose.pose.position.y = pose["y"]
                message.pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
                message.pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
                message.pose.covariance[0] = 0.25
                message.pose.covariance[7] = 0.25
                message.pose.covariance[35] = 0.0685
                self._initial_pose_pub.publish(message)

            deadline = time.monotonic() + self.INITIAL_POSE_TF_TIMEOUT_SEC
            while self._running and time.monotonic() < deadline and not self._has_map_transform():
                time.sleep(0.1)
            if not self._has_map_transform():
                raise RuntimeError("AMCL did not provide map -> base_footprint after initial pose.")

            with self._lock:
                self.navigation_mode_state = "ACTIVATING"
                self.navigation_ready_state = False

            current_states = [self._lifecycle_state(name, 1.0) for name in self.NAVIGATION_NODES]
            needs_reset = self._navigation_activation_attempted or any(
                state not in (None, 1) for state in current_states
            )
            if needs_reset and not self._manage_navigation_lifecycle(
                self._manage_lifecycle_type.Request.RESET, 30.0
            ):
                raise RuntimeError(
                    "Could not reset the partially started Nav2 stack: "
                    + self._navigation_lifecycle_summary()
                )
            self._navigation_activation_attempted = True
            if not self._manage_navigation_lifecycle(
                self._manage_lifecycle_type.Request.STARTUP,
                self.NAV2_STARTUP_TIMEOUT_SEC,
            ):
                raise RuntimeError(
                    "The Nav2 lifecycle manager could not activate the navigation nodes: "
                    + self._navigation_lifecycle_summary()
                )
            if not self._wait_for_lifecycle_nodes(
                self.NAVIGATION_NODES, self.NAV2_STARTUP_TIMEOUT_SEC
            ):
                inactive = [name for name in self.NAVIGATION_NODES if self._lifecycle_state(name) != 3]
                raise RuntimeError(f"Nav2 nodes are not active: {', '.join(inactive)}")
            if self._action_client is None or not self._action_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError("NavigateToPose action server is unavailable after activation.")
            with self._lock:
                self.navigation_ready_state = True
                self.navigation_mode_state = "READY"
                self.localization_state = "LOCALIZED"
                self.error = None
        except Exception as exc:
            self._publish_stop()
            with self._lock:
                self.navigation_ready_state = False
                self.navigation_mode_state = "ERROR"
                self.error = f"NAV2_ACTIVATION_FAILED: {exc}"
        finally:
            self._navigation_activation_lock.release()

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
        x, y, yaw = values
        if self._navigation_process is None or self._navigation_process.poll() is not None:
            return self._failure("Nav2 has not loaded a map yet. Wait for automatic map startup.")
        with self._lock:
            if self.navigation_state in {"SENDING", "NAVIGATING", "CANCELING"}:
                return self._failure("Cancel the active navigation goal before changing the initial pose.")
            self.localization_state = "LOCALIZING"
            self.navigation_ready_state = False
            self.navigation_mode_state = "ACTIVATING"
            self.error = None
            self._pose_map_path = self._navigation_map_path
        threading.Thread(
            target=self._activate_navigation_after_initial_pose,
            args=({"x": x, "y": y, "yaw": yaw},),
            daemon=True,
        ).start()
        return {"status": "accepted", "localization_state": "LOCALIZING"}

    def _start_navigation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        target = payload.get("target") or {}
        values = self._pose_values(target)
        if values is None:
            return self._failure("Navigation target requires finite x, y and theta values.")
        if self._node is None or self._action_client is None:
            return self._failure(self.error or "ROS is unavailable.")
        if not self.navigation_ready():
            return self._failure("Nav2 is not ready. Set the initial pose and wait for activation.")
        if self.use_sim_time and self._node.get_clock().now().nanoseconds <= 0:
            return self._failure(
                "SIM_CLOCK_UNAVAILABLE: No nonzero ROS simulated time has been received on /clock."
            )
        with self._lock:
            if self.navigation_state in {"SENDING", "NAVIGATING", "CANCELING"}:
                return self._failure("A navigation goal is already active.")
            self.active_goal_id = str(payload.get("command_id") or "")
            self.navigation_state = "SENDING"
            self.error = None

        if not self._action_client.server_is_ready():
            self._action_client.wait_for_server(timeout_sec=2.0)
        if not self._action_client.server_is_ready():
            with self._lock:
                self.navigation_state = "FAILED"
                self.error = "NAV2_UNAVAILABLE"
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
        future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        response_event = threading.Event()
        response_lock = threading.Lock()
        response: Dict[str, Any] = {"accepted": False, "timed_out": False}

        def handle_goal_response(done_future: Any) -> None:
            try:
                goal_handle = done_future.result()
                accepted = goal_handle is not None and goal_handle.accepted
                with response_lock:
                    response["accepted"] = accepted
                    timed_out = response["timed_out"]
                if timed_out:
                    if accepted:
                        goal_handle.cancel_goal_async()
                    return
                self._on_goal_response(done_future)
            except Exception as exc:
                response["error"] = str(exc)
                with self._lock:
                    self.navigation_state = "FAILED"
                    self.error = str(exc)
                    self.active_goal_id = None
            finally:
                response_event.set()

        future.add_done_callback(handle_goal_response)
        if not response_event.wait(self.GOAL_RESPONSE_TIMEOUT_SEC):
            with response_lock:
                if response_event.is_set():
                    timed_out = False
                else:
                    response["timed_out"] = True
                    timed_out = True
            if not timed_out:
                response_event.wait(0.1)
            else:
                with self._lock:
                    self.navigation_state = "FAILED"
                    self.error = "GOAL_RESPONSE_TIMEOUT"
                    self.active_goal_id = None
                return self._failure("Nav2 did not acknowledge the goal in time.")
        if not response["accepted"]:
            return self._failure(str(response.get("error") or "Nav2 rejected the navigation goal."))
        timeout_sec = max(1.0, min(float(payload.get("timeout_sec", 180)), 600.0))
        self._goal_timeout_timer = threading.Timer(
            timeout_sec,
            self._on_goal_timeout,
            args=(self.active_goal_id,),
        )
        self._goal_timeout_timer.daemon = True
        self._goal_timeout_timer.start()
        return {"status": "accepted", "command_id": self.active_goal_id}

    def _cancel_navigation(self, command_id: str) -> Dict[str, Any]:
        with self._lock:
            if command_id and self.active_goal_id and command_id != self.active_goal_id:
                return self._failure("The requested command is not the active goal.")
            goal_handle = self._goal_handle
            self.navigation_state = "CANCELING"
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        if self._goal_timeout_timer is not None:
            self._goal_timeout_timer.cancel()
        self._publish_stop()
        return {"status": "accepted", "command_id": self.active_goal_id}

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                if self._goal_timeout_timer is not None:
                    self._goal_timeout_timer.cancel()
                with self._lock:
                    self.navigation_state = "REJECTED"
                    self.error = "GOAL_REJECTED"
                    self.active_goal_id = None
                return
            with self._lock:
                self._goal_handle = goal_handle
                self.navigation_state = "NAVIGATING"
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_navigation_result)
        except Exception as exc:
            if self._goal_timeout_timer is not None:
                self._goal_timeout_timer.cancel()
            with self._lock:
                self.navigation_state = "FAILED"
                self.error = str(exc)
                self.active_goal_id = None

    def _on_navigation_result(self, future: Any) -> None:
        if self._goal_timeout_timer is not None:
            self._goal_timeout_timer.cancel()
        try:
            status = int(future.result().status)
            state = {4: "SUCCEEDED", 5: "CANCELED", 6: "FAILED"}.get(status, f"STATUS_{status}")
        except Exception as exc:
            state = "FAILED"
            self.error = str(exc)
        with self._lock:
            self.navigation_state = state
            self._goal_handle = None
            if state in {"SUCCEEDED", "CANCELED", "FAILED"}:
                self.active_goal_id = None

    def _on_goal_timeout(self, command_id: Optional[str]) -> None:
        with self._lock:
            if not command_id or self.active_goal_id != command_id:
                return
            goal_handle = self._goal_handle
            self.navigation_state = "FAILED"
            self.error = "TIMEOUT"
            self.active_goal_id = None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self._publish_stop()

    def _on_feedback(self, _: Any) -> None:
        with self._lock:
            self.navigation_state = "NAVIGATING"

    def _on_amcl_pose(self, message: Any) -> None:
        if self.runtime_id == "sim":
            return
        pose = message.pose.pose
        with self._lock:
            self.pose = self._message_pose(pose)
            self.localization_state = "LOCALIZED"
            if self._navigation_process is not None and self._navigation_process.poll() is None:
                self._pose_map_path = self._navigation_map_path

    def _update_pose_from_tf(self) -> None:
        if self._tf_buffer is None:
            return
        try:
            from rclpy.time import Time

            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            ).transform
        except Exception:
            return

        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        with self._lock:
            self.pose = {
                "x": float(transform.translation.x),
                "y": float(transform.translation.y),
                "yaw": yaw,
            }
            self.localization_state = "LOCALIZED"
            if self._navigation_process is not None and self._navigation_process.poll() is None:
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
            should_post = now - self._last_map_post >= 1.0
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
            navigation_active = self._navigation_process is not None and self._navigation_process.poll() is None
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
                "navigationReady": self.navigation_ready_state,
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

