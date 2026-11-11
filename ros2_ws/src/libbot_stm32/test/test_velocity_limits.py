import pytest

from libbot_stm32.velocity_limits import clamp_velocity


MAXIMUMS = (1.0, 1.0, 3.0)


@pytest.mark.parametrize(
    ('command', 'expected'),
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.0178, 0.0044, 0.0082), (0.0178, 0.0044, 0.0082)),
        ((-0.1, 0.0, 0.0), (-0.1, 0.0, 0.0)),
        ((0.0, -0.1, 0.0), (0.0, -0.1, 0.0)),
        ((0.0, 0.0, -0.1), (0.0, 0.0, -0.1)),
        ((0.1, 0.05, 0.2), (0.1, 0.05, 0.2)),
        ((2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((-2.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        ((0.0, 2.0, 0.0), (0.0, 1.0, 0.0)),
        ((0.0, 0.0, 4.0), (0.0, 0.0, 3.0)),
    ],
)
def test_clamp_velocity_preserves_continuous_commands(command, expected):
    assert clamp_velocity(*command, maximums=MAXIMUMS) == expected
