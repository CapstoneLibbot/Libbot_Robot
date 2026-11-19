import pytest

from libbot_stm32.wheel_diagnostics import WheelResponseMonitor, mecanum_targets


def test_mecanum_targets_allow_valid_wheel_cancellation():
    assert mecanum_targets(0.04, 0.04, 0.0) == pytest.approx((0.08, 0, 0, 0.08))
    assert mecanum_targets(0, 0.1, 0) == pytest.approx((0.1, -0.1, -0.1, 0.1))
    assert mecanum_targets(0, 0, 0.2) == pytest.approx((0.06, -0.06, 0.06, -0.06))


def test_sustained_single_encoder_response_is_reported():
    monitor = WheelResponseMonitor(interval=1.0)
    assert monitor.update(0, (0, 0.1, 0), 0, (0, 0, 0, 0)) is None
    result = monitor.update(1.0, (0, 0.1, 0), 0, (0.05, 0, 0, 0))
    assert result['moving'] == (1,)
    assert result['targets'] == pytest.approx((0.1, -0.1, -0.1, 0.1))


def test_normal_two_wheel_movement_and_sensor_block_are_not_reported():
    monitor = WheelResponseMonitor(interval=1.0)
    monitor.update(0, (0.04, 0.04, 0), 0, (0, 0, 0, 0))
    assert monitor.update(1, (0.04, 0.04, 0), 0, (0.03, 0, 0, 0.03)) is None
    monitor.update(2, (0, 0.1, 0), 0, (0, 0, 0, 0))
    assert monitor.update(3, (0, 0.1, 0), 1, (0.1, 0, 0, 0)) is None
    assert monitor.update(4, None, 0, (0.1, 0, 0, 0)) is None


def test_direction_change_restarts_observation():
    monitor = WheelResponseMonitor(interval=1.0)
    monitor.update(0, (0, 0.1, 0), 0, (0, 0, 0, 0))
    assert monitor.update(0.9, (0, -0.1, 0), 0, (0.1, 0, 0, 0)) is None
    assert monitor.update(1.1, (0, -0.1, 0), 0, (0.01, 0, 0, 0)) is None
