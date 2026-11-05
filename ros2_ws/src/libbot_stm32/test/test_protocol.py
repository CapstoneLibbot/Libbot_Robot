import pytest

from libbot_stm32.protocol import parse_imu_line, parse_range_line


def test_parse_imu_packet():
    values = parse_imu_line(
        'IMU,16384,0,0,0,1,-2,3,0,0,9810,2,3,2,1'
    )
    assert values[0:4] == (16384, 0, 0, 0)
    assert values[10:14] == (2, 3, 2, 1)


@pytest.mark.parametrize(
    'line',
    [
        'IMU,1,2,3',
        'IMU,16384,0,0,0,0,0,0,0,0,9810,4,3,3,3',
        'IMU,not-a-number,0,0,0,0,0,0,0,0,0,0,0,0,0',
    ],
)
def test_reject_invalid_imu_packets(line):
    with pytest.raises(ValueError):
        parse_imu_line(line)


def test_parse_range_packet():
    distances, valid, blocked = parse_range_line('RNG,200,300,-1,90,11,8')
    assert distances == (200, 300, -1, 90)
    assert valid == 0b1011
    assert blocked == 0b1000


@pytest.mark.parametrize(
    'line',
    [
        'RNG,1,2,3',
        'RNG,200,300,400,500,16,0',
        'RNG,200,300,400,999999999999999999999,15,0',
    ],
)
def test_reject_invalid_range_packets(line):
    with pytest.raises(ValueError):
        parse_range_line(line)
