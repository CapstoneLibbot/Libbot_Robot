"""Observe wheel response without changing the robot's velocity commands."""

import math


def mecanum_targets(vx, vy, wz, lever_arm=0.30, max_wheel_speed=0.40):
    """Mirror the STM32 mecanum mix and proportional wheel-speed limit."""
    yaw = lever_arm * wz
    targets = (vx + vy + yaw, vx - vy - yaw,
               vx - vy + yaw, vx + vy - yaw)
    peak = max(abs(value) for value in targets)
    scale = min(1.0, max_wheel_speed / peak) if peak else 1.0
    return tuple(value * scale for value in targets)


class WheelResponseMonitor:
    """Warn about a sustained discrepancy; not a motor safety controller."""

    def __init__(self, interval=1.0, min_target=0.02, min_displacement=0.005):
        self.interval = interval
        self.min_target = min_target
        self.min_displacement = min_displacement
        self.reset()

    def reset(self):
        self.started = None
        self.signature = None
        self.distance = [0.0] * 4

    def update(self, now, command, blocked_mask, wheel_distances):
        # STM32 may independently suppress components while a sensor is blocked.
        if command is None or blocked_mask != 0 or not all(
                math.isfinite(value) for value in (*command, *wheel_distances)):
            self.reset()
            return None
        targets = mecanum_targets(*command)
        signature = tuple(1 if value >= self.min_target else -1 if value <= -self.min_target else 0
                          for value in targets)
        if sum(value != 0 for value in signature) < 2:
            self.reset()
            return None
        if self.started is None or signature != self.signature or now < self.started:
            self.reset()
            self.started = now
            self.signature = signature
            return None
        for i, distance in enumerate(wheel_distances):
            self.distance[i] += abs(distance)
        elapsed = now - self.started
        if elapsed < self.interval:
            return None
        moving = tuple(index + 1 for index, distance in enumerate(self.distance)
                       if distance >= self.min_displacement)
        result = None
        if len(moving) == 1:
            result = {'targets': targets, 'distance': tuple(self.distance),
                      'moving': moving, 'elapsed': elapsed}
        self.reset()
        return result
