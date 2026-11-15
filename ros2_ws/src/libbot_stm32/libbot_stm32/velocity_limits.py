"""Continuous velocity limits for commands forwarded to the STM32."""


def clamp_velocity(vx, vy, wz, maximums):
    """Clamp each velocity to its maximum without imposing a minimum.

    Nav2 needs small, continuous velocity commands to track a path and settle
    at a goal. Static-friction compensation belongs in the motor PWM control.
    """
    values = (vx, vy, wz)
    return tuple(
        max(-maximum, min(maximum, value))
        for value, maximum in zip(values, maximums)
    )
