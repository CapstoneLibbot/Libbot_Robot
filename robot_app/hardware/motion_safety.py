from __future__ import annotations

import time
from typing import Any, Dict, Optional


class MotionSafetyController:
    """Stops robot motion before blocking workflow operations."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}
        self.method = self.settings.get("method", "ros_topic")
        self.enabled = self._read_bool(self.settings.get("enabled", True))

    def stop(self, reason: str = "workflow_wait") -> Dict[str, Any]:
        started_at = time.monotonic()
        if not self.enabled or self.method == "disabled":
            return self._result("disabled", reason, started_at)
        if self.method == "serial":
            return self._stop_serial(reason, started_at)
        return self._stop_ros_topic(reason, started_at)

    def _stop_ros_topic(self, reason: str, started_at: float) -> Dict[str, Any]:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
        except ImportError:
            return self._result(
                "unavailable",
                reason,
                started_at,
                "rclpy and geometry_msgs are required to publish /cmd_vel.",
            )

        node = None
        initialized_here = False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                initialized_here = True
            node = rclpy.create_node("libot_workflow_motion_safety")
            publisher = node.create_publisher(Twist, self.settings.get("topic", "/cmd_vel"), 10)
            stop_msg = Twist()
            repeat = int(self.settings.get("repeat", 3))
            interval_sec = float(self.settings.get("interval_sec", 0.05))
            for _ in range(max(repeat, 1)):
                publisher.publish(stop_msg)
                rclpy.spin_once(node, timeout_sec=interval_sec)
            return self._result("sent", reason, started_at)
        except Exception as exc:
            return self._result("error", reason, started_at, str(exc))
        finally:
            if node is not None:
                node.destroy_node()
            if initialized_here and rclpy.ok():
                rclpy.shutdown()

    def _stop_serial(self, reason: str, started_at: float) -> Dict[str, Any]:
        try:
            import serial
        except ImportError:
            return self._result(
                "unavailable",
                reason,
                started_at,
                "pyserial is required for direct STM32 serial stop.",
            )

        port = self.settings.get("port", "/dev/serial0")
        baudrate = int(self.settings.get("baudrate", 115200))
        try:
            with serial.Serial(port, baudrate, timeout=0.1, write_timeout=0.1) as device:
                written = device.write(b"VEL,0,0,0\n")
            return self._result("sent", reason, started_at, f"written={written}")
        except (OSError, serial.SerialException) as exc:
            return self._result("error", reason, started_at, str(exc))

    def _result(
        self,
        status: str,
        reason: str,
        started_at: float,
        message: Optional[str] = None,
    ) -> Dict[str, Any]:
        result = {
            "status": status,
            "method": self.method,
            "reason": reason,
            "duration_ms": int((time.monotonic() - started_at) * 1000),
        }
        if message:
            result["message"] = message
        return result

    def _read_bool(self, value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() not in {"0", "false", "no", "off"}
        return bool(value)
