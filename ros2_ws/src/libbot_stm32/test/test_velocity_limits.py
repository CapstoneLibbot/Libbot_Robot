import pytest

from libbot_stm32.velocity_limits import clamp_body_velocity


MINIMUMS = (0.2, 0.2, 0.7)
MAXIMUMS = (1.0, 1.0, 3.0)


@pytest.mark.parametrize(
    ('command', 'expected'),
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.1, 0.0, 0.0), (0.2, 0.0, 0.0)),
        ((-0.1, 0.0, 0.0), (-0.2, 0.0, 0.0)),
        ((0.0, 0.1, 0.0), (0.0, 0.2, 0.0)),
        ((0.0, -0.1, 0.0), (0.0, -0.2, 0.0)),
        ((0.0, 0.0, 0.1), (0.0, 0.0, 0.7)),
        ((0.0, 0.0, -0.1), (0.0, 0.0, -0.7)),
        # Forward motion dominates a small angular correction.
        ((0.15, 0.0, 0.1), (0.2, 0.0, 0.0)),
        # A larger intended turn dominates a small forward correction.
        ((0.1, 0.0, 0.4), (0.0, 0.0, 0.7)),
        # Sub-minimum secondary axes are removed; valid axes are preserved.
        ((0.3, 0.25, 0.1), (0.3, 0.25, 0.0)),
        ((2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((-2.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        ((0.0, 0.0, 4.0), (0.0, 0.0, 3.0)),
        # Equal normalized magnitudes resolve deterministically to x, y, wz.
        ((0.1, 0.1, 0.0), (0.2, 0.0, 0.0)),
    ],
)
def test_clamp_body_velocity(command, expected):
    assert clamp_body_velocity(
        *command, minimums=MINIMUMS, maximums=MAXIMUMS
    ) == expected
