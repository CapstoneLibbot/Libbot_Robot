"""Pure parsing helpers for the line-oriented STM32 protocol."""


def _parse_ints(line, prefix, count):
    parts = line.split(',')
    if len(parts) != count + 1 or parts[0] != prefix:
        raise ValueError(f'invalid {prefix} field count')
    return tuple(int(value) for value in parts[1:])


def parse_imu_line(line):
    values = _parse_ints(line, 'IMU', 14)
    calibration = values[10:14]
    if any(value < 0 or value > 3 for value in calibration):
        raise ValueError('IMU calibration value outside 0..3')
    return values


def parse_range_line(line):
    values = _parse_ints(line, 'RNG', 6)
    distances = values[:4]
    valid_mask, blocked_mask = values[4:]
    if valid_mask < 0 or valid_mask > 0x0F:
        raise ValueError('range valid mask outside 0..15')
    if blocked_mask < 0 or blocked_mask > 0x0F:
        raise ValueError('range blocked mask outside 0..15')
    if any(value < -1 or value > 4000 for value in distances):
        raise ValueError('range distance outside supported bounds')
    return distances, valid_mask, blocked_mask
