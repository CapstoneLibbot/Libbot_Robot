"""Velocity command limits shared by the STM32 UART bridge."""


def clamp_body_velocity(vx, vy, wz, minimums, maximums):
    """Clamp a body command without promoting minor axes to minimum speed.

    The strongest axis relative to its minimum speed is considered the
    intended motion. A non-zero sub-minimum command on that axis is raised to
    its minimum; sub-minimum commands on other axes are discarded. This keeps
    small controller corrections from becoming large unintended motions.
    """
    values = [vx, vy, wz]
    values = [
        max(-maximum, min(maximum, value))
        for value, maximum in zip(values, maximums)
    ]

    active_axes = [i for i, value in enumerate(values) if value != 0.0]
    if not active_axes:
        return 0.0, 0.0, 0.0

    dominant_axis = max(
        active_axes,
        key=lambda i: abs(values[i]) / minimums[i],
    )

    for i, value in enumerate(values):
        minimum = minimums[i]
        if value == 0.0 or abs(value) >= minimum:
            continue
        if i == dominant_axis:
            values[i] = minimum if value > 0.0 else -minimum
        else:
            values[i] = 0.0

    return tuple(values)
