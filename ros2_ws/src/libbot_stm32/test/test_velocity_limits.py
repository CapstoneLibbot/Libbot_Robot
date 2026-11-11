import pytest

from libbot_stm32.velocity_limits import clamp_with_minimum


@pytest.mark.parametrize(
    ('value', 'minimum', 'maximum', 'expected'),
    [
        (0.1, 0.2, 1.0, 0.2),
        (-0.1, 0.2, 1.0, -0.2),
        (0.0, 0.2, 1.0, 0.0),
        (0.2, 0.2, 1.0, 0.2),
        (-0.2, 0.2, 1.0, -0.2),
        (0.7, 0.2, 1.0, 0.7),
        (-0.7, 0.2, 1.0, -0.7),
        (2.0, 0.2, 1.0, 1.0),
        (-2.0, 0.2, 1.0, -1.0),
        (0.01, 0.7, 3.0, 0.7),
        (-0.01, 0.7, 3.0, -0.7),
    ],
)
def test_clamp_with_minimum(value, minimum, maximum, expected):
    assert clamp_with_minimum(value, minimum, maximum) == expected
