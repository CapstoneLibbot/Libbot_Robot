"""Velocity command limits shared by the STM32 UART bridge."""


def clamp_with_minimum(value, minimum, maximum):
    """Clamp magnitude to the configured maximum and floor non-zero values."""
    value = max(-maximum, min(maximum, value))

    if 0.0 < value < minimum:
        return minimum
    if -minimum < value < 0.0:
        return -minimum
    return value
