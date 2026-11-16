#!/usr/bin/env python3
"""Build a simple Gazebo world from a Nav2 occupancy-grid PGM/YAML map."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    data = path.read_bytes()
    offset = 0

    def token() -> bytes:
        nonlocal offset
        while offset < len(data):
            if data[offset] == ord("#"):
                newline = data.find(b"\n", offset)
                offset = len(data) if newline < 0 else newline + 1
            elif data[offset] in b" \t\r\n":
                offset += 1
            else:
                break
        start = offset
        while offset < len(data) and data[offset] not in b" \t\r\n#":
            offset += 1
        value = data[start:offset]
        if not value:
            raise ValueError(f"Unexpected end of PGM header: {path}")
        return value

    magic = token()
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"Unsupported PGM format {magic!r}; expected P2 or P5")
    width, height, max_value = int(token()), int(token()), int(token())
    if width <= 0 or height <= 0 or not 0 < max_value <= 65535:
        raise ValueError(f"Invalid PGM dimensions or max value in {path}")

    if magic == b"P2":
        samples = [int(token()) for _ in range(width * height)]
    else:
        # Consume the whitespace separating maxval from the binary raster.
        if offset >= len(data) or data[offset] not in b" \t\r\n":
            raise ValueError(f"Missing PGM raster separator: {path}")
        separator = data[offset:offset + 2]
        offset += 2 if separator == b"\r\n" else 1
        bytes_per_sample = 1 if max_value < 256 else 2
        raster = data[offset:]
        expected = width * height * bytes_per_sample
        if len(raster) < expected:
            raise ValueError(f"Truncated PGM raster in {path}")
        if bytes_per_sample == 1:
            samples = list(raster[:expected])
        else:
            samples = [int.from_bytes(raster[i:i + 2], "big") for i in range(0, expected, 2)]

    if any(sample < 0 or sample > max_value for sample in samples):
        raise ValueError(f"PGM sample is outside [0, {max_value}] in {path}")
    if max_value != 255:
        samples = [round(sample * 255 / max_value) for sample in samples]
    return width, height, samples


def fill_enclosed_unknown(width: int, height: int, occupied: list[bool], unknown: list[bool]) -> list[bool]:
    """Treat unknown islands fully enclosed by occupied cells as solid obstacles."""
    visited = [False] * (width * height)
    result = occupied.copy()
    for start, is_unknown in enumerate(unknown):
        if not is_unknown or visited[start]:
            continue
        stack = [start]
        visited[start] = True
        component: list[int] = []
        touches_free = False
        while stack:
            index = stack.pop()
            component.append(index)
            row, col = divmod(index, width)
            for nr, nc in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
                if not (0 <= nr < height and 0 <= nc < width):
                    continue
                neighbor = nr * width + nc
                if not occupied[neighbor] and not unknown[neighbor]:
                    touches_free = True
                elif unknown[neighbor] and not visited[neighbor]:
                    visited[neighbor] = True
                    stack.append(neighbor)
        if not touches_free:
            for index in component:
                result[index] = True
    return result


def merged_rectangles(width: int, height: int, mask: list[bool]) -> list[tuple[int, int, int, int]]:
    """Merge occupied raster cells into rectangles (row0, row1, col0, col1)."""
    finished: list[tuple[int, int, int, int]] = []
    active: dict[tuple[int, int], int] = {}
    for row in range(height):
        runs: list[tuple[int, int]] = []
        col = 0
        while col < width:
            if not mask[row * width + col]:
                col += 1
                continue
            start = col
            while col < width and mask[row * width + col]:
                col += 1
            runs.append((start, col))
        current = set(runs)
        for run, start_row in list(active.items()):
            if run not in current:
                finished.append((start_row, row, run[0], run[1]))
                del active[run]
        for run in runs:
            active.setdefault(run, row)
    for run, start_row in active.items():
        finished.append((start_row, height, run[0], run[1]))
    return finished


def add_box(parent: ET.Element, name: str, size: tuple[float, float, float], center: tuple[float, float, float], color: str) -> None:
    model = ET.SubElement(parent, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{center[0]:.6f} {center[1]:.6f} {center[2]:.6f} 0 0 0"
    link = ET.SubElement(model, "link", {"name": "link"})
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = " ".join(f"{value:.6f}" for value in size)
        if tag == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color


def create_world(map_yaml: Path, world_path: Path, obstacle_height: float) -> tuple[int, int, int]:
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = (map_yaml.parent / metadata["image"]).resolve()
    width, height, pixels = read_pgm(image_path)
    resolution = float(metadata["resolution"])
    origin_x, origin_y, origin_yaw = map(float, metadata["origin"])
    if abs(origin_yaw) > 1e-8:
        raise ValueError("Rotated map origins are not supported; origin yaw must be zero")
    negate = int(metadata.get("negate", 0))
    occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    free_threshold = float(metadata.get("free_thresh", 0.196))
    probabilities = [sample / 255.0 if negate else (255 - sample) / 255.0 for sample in pixels]
    occupied = [value > occupied_threshold for value in probabilities]
    unknown = [free_threshold < value < occupied_threshold for value in probabilities]
    obstacles = fill_enclosed_unknown(width, height, occupied, unknown)
    rectangles = merged_rectangles(width, height, obstacles)

    sdf = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(sdf, "world", {"name": "map_world"})
    physics = ET.SubElement(world, "physics", {"name": "physics", "type": "dart"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
    ):
        plugin = ET.SubElement(world, "plugin", {"filename": filename, "name": name})
        if "sensors" in filename:
            ET.SubElement(plugin, "render_engine").text = "ogre2"

    light = ET.SubElement(world, "light", {"type": "directional", "name": "sun"})
    ET.SubElement(light, "pose").text = "0 0 8 0 0 0"
    ET.SubElement(light, "direction").text = "-0.5 0.4 -1"

    map_width, map_height = width * resolution, height * resolution
    floor_center_x = origin_x + map_width / 2
    floor_center_y = origin_y + map_height / 2
    add_box(world, "map_floor", (map_width + 0.5, map_height + 0.5, 0.1), (floor_center_x, floor_center_y, -0.05), "0.72 0.69 0.62 1")

    for index, (row0, row1, col0, col1) in enumerate(rectangles):
        size_x = (col1 - col0) * resolution
        size_y = (row1 - row0) * resolution
        center_x = origin_x + (col0 + col1) * resolution / 2
        center_y = origin_y + (height - (row0 + row1) / 2) * resolution
        add_box(world, f"map_obstacle_{index:04d}", (size_x, size_y, obstacle_height), (center_x, center_y, obstacle_height / 2), "0.40 0.39 0.36 1")

    ET.indent(sdf, space="  ")
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(ET.tostring(sdf, encoding="unicode", xml_declaration=True) + "\n", encoding="utf-8")
    return width, height, len(rectangles)


def main() -> None:
    robot_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, help="Nav2 map YAML")
    parser.add_argument("--world", type=Path, help="Output SDF path")
    parser.add_argument("--obstacle-height", type=float, default=2.0)
    args = parser.parse_args()
    if args.obstacle_height <= 0 or not math.isfinite(args.obstacle_height):
        parser.error("--obstacle-height must be a positive finite number")
    world_path = args.world or robot_dir / "ros2_ws" / "src" / "libot_description" / "worlds" / f"{args.map.stem}_gazebo.sdf"
    width, height, count = create_world(args.map.resolve(), world_path.resolve(), args.obstacle_height)
    print(f"Generated {world_path} from {args.map}: {width}x{height} cells, {count} obstacle boxes")


if __name__ == "__main__":
    main()
