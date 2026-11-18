from __future__ import annotations

import math
import threading
import time
import uuid
from typing import Any, Dict


def _bounded_correction(error: float, tolerance: float, gain: float, minimum: float, maximum: float) -> float:
    """Apply a small bounded correction only while outside the stop tolerance."""
    if abs(error) <= tolerance:
        return 0.0
    return math.copysign(min(maximum, max(minimum, abs(error) * gain)), error)


class ShelfMotionController:
    """Move between scan positions through Nav2's velocity safety pipeline."""

    def __init__(self, gateway: Any, settings: Dict[str, Any] | None = None):
        self.gateway = gateway
        self.settings = settings or {}
        self.cancelled = threading.Event()

    def stop(self) -> None:
        self.cancelled.set()
        if self.gateway is not None:
            self.gateway._publish_stop()

    def wait_stopped(self, context: Any) -> None:
        if self.gateway is None:
            raise RuntimeError("Robot odometry is unavailable.")
        deadline, stationary_since = time.monotonic() + 5.0, None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if self.cancelled.is_set() or context.get("workflow_cancelled") or context.get("low_battery_preempt_requested"):
                raise RuntimeError("Shelf scan cancelled.")
            with self.gateway._lock:
                odom_age = now - (self.gateway._last_odom_monotonic or 0)
                velocity = dict(self.gateway.twist)
            if odom_age <= 1.0 and all(math.isfinite(float(velocity.get(key, float('nan')))) and abs(float(velocity[key])) < limit
                                       for key, limit in (("linearX", 0.01), ("linearY", 0.01), ("angularZ", 0.03))):
                stationary_since = stationary_since or now
                if now - stationary_since >= 0.3:
                    return
            else:
                stationary_since = None
            time.sleep(0.05)
        raise RuntimeError("Robot did not stop or odometry is stale.")

    def move_to(self, target: Dict[str, float], context: Any) -> None:
        if self.gateway is None or self.gateway.localization_state != "LOCALIZED":
            raise RuntimeError("Shelf scan requires a localized runtime gateway.")
        import rclpy
        from geometry_msgs.msg import Twist
        from lifecycle_msgs.srv import GetState
        from rclpy.time import Time

        speed = float(self.settings.get("speed_mps", 0.5))
        min_speed = float(self.settings.get("min_speed_mps", min(0.02, speed)))
        min_angular_speed = float(self.settings.get("min_angular_speed_radps", 0.08))
        if (not math.isfinite(speed) or not 0 < speed <= 0.5
                or not math.isfinite(min_speed) or not 0 <= min_speed <= speed
                or not math.isfinite(min_angular_speed) or not 0 <= min_angular_speed <= 0.15):
            raise ValueError("Invalid shelf scan speed or minimum correction speed.")
        deadline = time.monotonic() + float(self.settings.get("segment_timeout_sec", 20.0))
        node = rclpy.create_node(f"libot_shelf_motion_{uuid.uuid4().hex[:8]}")
        publisher = node.create_publisher(Twist, "/cmd_vel_nav", 10)
        client = node.create_client(GetState, "/collision_monitor/get_state")
        last_errors = None
        try:
            if not client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError("Collision monitor is unavailable.")
            next_check = 0.0
            state_future = None
            while time.monotonic() < deadline:
                now = time.monotonic()
                if self.cancelled.is_set() or context.get("workflow_cancelled") or context.get("low_battery_preempt_requested"):
                    raise RuntimeError("Shelf scan cancelled.")
                if self.gateway.localization_state != "LOCALIZED":
                    raise RuntimeError("Localization was lost during shelf scan.")
                if now - (self.gateway._last_scan_monotonic or 0) > 1.0:
                    raise RuntimeError("LiDAR data is stale during shelf scan.")
                if state_future is None and now >= next_check:
                    state_future = client.call_async(GetState.Request())
                    next_check = now + 0.5
                rclpy.spin_once(node, timeout_sec=0.05)
                if state_future is not None:
                    if state_future.done():
                        if state_future.result() is None or state_future.result().current_state.label != "active":
                            raise RuntimeError("Collision monitor is not active.")
                        state_future = None
                    elif now >= next_check:
                        raise RuntimeError("Collision monitor state timed out.")
                    else:
                        publisher.publish(Twist())
                        continue
                try:
                    stamped = self.gateway._tf_buffer.lookup_transform("map", "base_footprint", Time())
                    stamp_ns = stamped.header.stamp.sec * 1_000_000_000 + stamped.header.stamp.nanosec
                    age = (self.gateway._node.get_clock().now().nanoseconds - stamp_ns) / 1e9
                    if stamp_ns <= 0 or age > 1.0 or age < -0.2:
                        raise ValueError("stale transform")
                    transform = stamped.transform
                    q = transform.rotation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    dx, dy = target["x"] - transform.translation.x, target["y"] - transform.translation.y
                    heading_error = math.atan2(math.sin(target["theta"] - yaw), math.cos(target["theta"] - yaw))
                    if not all(math.isfinite(v) for v in (dx, dy, heading_error)):
                        raise ValueError("non-finite pose")
                except Exception as exc:
                    raise RuntimeError(f"Shelf scan pose unavailable: {exc}") from exc
                if abs(heading_error) > 0.45:
                    raise RuntimeError("Robot heading deviated from the shelf scan orientation.")
                for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
                    point = {"x": transform.translation.x + dx * fraction,
                             "y": transform.translation.y + dy * fraction}
                    if self.gateway.navigation_zone_blocking_action("GOAL", point):
                        raise RuntimeError("Shelf scan route enters a restricted navigation zone.")
                forward = math.cos(yaw) * dx + math.sin(yaw) * dy
                lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
                last_errors = (forward, lateral, heading_error)
                if abs(forward) > 0.08:
                    raise RuntimeError("Robot deviated from the configured shelf scan line.")
                if abs(lateral) <= 0.015 and abs(heading_error) <= 0.04:
                    return
                msg = Twist()
                if abs(heading_error) <= 0.12:
                    msg.linear.y = _bounded_correction(lateral, 0.015, 0.7, min_speed, speed)
                msg.angular.z = _bounded_correction(heading_error, 0.04, 0.8, min_angular_speed, 0.15)
                publisher.publish(msg)
            details = ""
            if last_errors is not None:
                details = (f" Last errors: forward={last_errors[0]:.3f} m, "
                           f"lateral={last_errors[1]:.3f} m, heading={last_errors[2]:.3f} rad.")
            raise RuntimeError("Shelf scan movement timed out; check motion and safety pipeline." + details)
        finally:
            for _ in range(3):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
