from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional
from urllib import request


class RobotRuntimeGateway:
    """ROS-facing runtime adapter used by both Gazebo and the physical robot."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        config = settings or {}
        runtime = config.get("runtime", {})
        self.runtime_id = str(
            os.getenv("LIBOT_RUNTIME_ID", runtime.get("id", "real"))
        ).strip().lower()
        self.use_sim_time = self.runtime_id == "sim"
        self.backend_url = os.getenv(
            "LIBOT_BACKEND_URL",
            runtime.get("backend_url", "http://127.0.0.1:5205"),
        ).rstrip("/")
        self.telemetry_hz = max(1.0, min(float(runtime.get("telemetry_hz", 5.0)), 10.0))
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.twist = {"linearX": 0.0, "linearY": 0.0, "angularZ": 0.0}
        self.battery: Optional[float] = None
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
        self._running = False
        self._last_telemetry_error_log = 0.0
        self._map: Optional[Dict[str, Any]] = None
        self._last_map_post = 0.0
        self._manual_active = False
        self._last_manual_command = 0.0
        self._mapping_process: Optional[subprocess.Popen[Any]] = None
        self._navigation_process: Optional[subprocess.Popen[Any]] = None

    def start(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from nav2_msgs.action import NavigateToPose
            from nav_msgs.msg import OccupancyGrid, Odometry, Path
            from rclpy.action import ActionClient
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.parameter import Parameter
            from sensor_msgs.msg import BatteryState
            from tf2_ros import Buffer, TransformListener
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
        self._node.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        self._node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self._node.create_subscription(Path, "/plan", self._on_plan, 10)
        self._node.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self._tf_buffer = Buffer(node=self._node)
        self._tf_listener = TransformListener(self._tf_buffer, self._node)
        self._tf_timer = self._node.create_timer(0.1, self._update_pose_from_tf)
        self._manual_watchdog_timer = self._node.create_timer(0.05, self._manual_watchdog)
        self._action_client = ActionClient(self._node, NavigateToPose, "/navigate_to_pose")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._running = True
        threading.Thread(target=self._executor.spin, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        self._publish_stop()
        self._terminate_process(self._mapping_process)
        self._terminate_process(self._navigation_process)
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        if self._node is not None:
            self._node.destroy_node()

    def handle_request(self, payload: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(payload, dict):
            return None
        command = str(payload.get("command", "")).lower()
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
            self._publish_stop()
            with self._lock:
                self._manual_active = False
            return {"status": "accepted"}
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
            return self._start_navigation_mode(str(payload.get("map_path", "")))
        if command == "stop_navigation_mode":
            self._terminate_process(self._navigation_process)
            self._navigation_process = None
            self._publish_stop()
            return {"status": "accepted"}
        return None

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
            if not all(math.isfinite(value) for value in values):
                raise ValueError("Velocity must be finite.")
        except (TypeError, ValueError) as exc:
            return self._failure(str(exc))
        from geometry_msgs.msg import Twist

        message = Twist()
        message.linear.x = max(-0.7, min(0.7, values[0]))
        message.linear.y = max(-0.7, min(0.7, values[1]))
        message.angular.z = max(-0.5, min(0.5, values[2]))
        self._stop_pub.publish(message)
        with self._lock:
            self._manual_active = True
            self._last_manual_command = time.monotonic()
        return {"status": "accepted"}

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
            return {"status": "accepted"}
        except OSError as exc:
            return self._failure(f"Could not start SLAM mapping: {exc}")

    def _start_navigation_mode(self, map_path: str) -> Dict[str, Any]:
        if self._mapping_process is not None and self._mapping_process.poll() is None:
            return self._failure("Stop SLAM mapping before starting Nav2.")
        path = Path(map_path)
        if not path.is_absolute() or path.suffix.lower() != ".yaml" or not str(path).startswith("/ros2_ws/maps/") or not path.is_file():
            return self._failure("The selected map YAML is not available on the robot.")
        if self._navigation_process is not None and self._navigation_process.poll() is None:
            return self._failure("Nav2 is already running.")
        try:
            with self._lock:
                initial_pose = dict(self.pose)
            self._navigation_process = subprocess.Popen(
                [
                    "ros2", "launch", "libot_bringup", "navigation.launch.py",
                    f"map:={path}",
                    "use_hardware:=false",
                    "start_workflow_app:=false",
                    "use_rviz:=false",
                    f"use_sim_time:={'true' if self.use_sim_time else 'false'}",
                ],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            time.sleep(0.25)
            if self._navigation_process.poll() is not None:
                self._navigation_process = None
                return self._failure("Could not start Nav2 with the selected map.")
            threading.Thread(target=self._seed_nav2_pose, args=(initial_pose,), daemon=True).start()
            return {"status": "accepted"}
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

    def _save_map(self, file_stem: str) -> Dict[str, Any]:
        with self._lock:
            grid = dict(self._map) if self._map is not None else None
        if grid is None:
            return self._failure("No live occupancy grid has been received.")
        if not file_stem.startswith("/ros2_ws/maps/") or ".." in Path(file_stem).parts:
            return self._failure("Map file path is invalid.")
        width = int(grid["width"])
        height = int(grid["height"])
        values = grid["data"]
        if width <= 0 or height <= 0 or len(values) != width * height:
            return self._failure("Occupancy grid dimensions do not match its data.")
        output = Path(file_stem)
        output.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = output.with_suffix(".pgm")
        yaml_path = output.with_suffix(".yaml")
        try:
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
        from geometry_msgs.msg import PoseWithCovarianceStamped

        x, y, yaw = values
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
        with self._lock:
            self.localization_state = "LOCALIZING"
            self.error = None
        return {"status": "accepted", "localization_state": "LOCALIZING"}

    def _start_navigation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        target = payload.get("target") or {}
        values = self._pose_values(target)
        if values is None:
            return self._failure("Navigation target requires finite x, y and theta values.")
        if self._node is None or self._action_client is None:
            return self._failure(self.error or "ROS is unavailable.")
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
        future.add_done_callback(self._on_goal_response)
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
                if now - self._last_telemetry_error_log >= 5.0:
                    print(f"Map delivery failed: {exc}", flush=True)
                    self._last_telemetry_error_log = now

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
        with request.urlopen(req, timeout=10.0):
            pass

    def _on_plan(self, message: Any) -> None:
        points = [{"x": float(p.pose.position.x), "y": float(p.pose.position.y)} for p in message.poses]
        with self._lock:
            self.planned_path = points[:500]

    def _on_battery(self, message: Any) -> None:
        percentage = float(message.percentage)
        with self._lock:
            self.battery = percentage * 100.0 if 0.0 <= percentage <= 1.0 else percentage

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

    def _post_telemetry(self) -> None:
        with self._lock:
            mapping_active = self._mapping_process is not None and self._mapping_process.poll() is None
            navigation_active = self._navigation_process is not None and self._navigation_process.poll() is None
            payload = {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "pose": dict(self.pose),
                "twist": dict(self.twist),
                "battery": self.battery,
                "localizationState": self.localization_state,
                "navigationState": self.navigation_state,
                "activeGoalId": self.active_goal_id,
                "plannedPath": list(self.planned_path),
                "error": self.error,
                "mappingActive": mapping_active,
                "navigationActive": navigation_active,
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

