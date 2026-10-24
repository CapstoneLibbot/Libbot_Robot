#!/usr/bin/env python3
"""Generate a Gazebo world and browser scene metadata from a ROS occupancy map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

FLOOR_THICKNESS_METERS = 0.1
OBSTACLE_HEIGHT_METERS = 1.0


def read_yaml(path: Path) -> dict:
    result: dict = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        if value.startswith("["):
            result[key] = [float(item.strip()) for item in value.strip("[]").split(",")]
        else:
            try:
                result[key] = float(value)
            except ValueError:
                result[key] = value
    return result


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    with path.open("rb") as stream:
        tokens: list[bytes] = []
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                raise ValueError("Invalid PGM header")
            if line.startswith(b"#"):
                continue
            tokens.extend(line.split())
        if tokens[0] != b"P5":
            raise ValueError("Only binary P5 PGM maps are supported")
        width, height, maximum = map(int, tokens[1:4])
        pixels = stream.read(width * height)
        if len(pixels) != width * height:
            raise ValueError("PGM pixel data is incomplete")
        return width, height, maximum, pixels


def occupied_rectangles(width: int, height: int, pixels: bytes, threshold: int) -> list[dict]:
    active: dict[tuple[int, int], dict] = {}
    complete: list[dict] = []
    for row in range(height):
        runs: list[tuple[int, int]] = []
        start = None
        for column in range(width + 1):
            occupied = column < width and pixels[row * width + column] <= threshold
            if occupied and start is None:
                start = column
            elif not occupied and start is not None:
                runs.append((start, column - start))
                start = None
        next_active: dict[tuple[int, int], dict] = {}
        for run in runs:
            rectangle = active.pop(run, None)
            if rectangle is None:
                rectangle = {"pixelX": run[0], "pixelY": row, "pixelWidth": run[1], "pixelHeight": 1}
            else:
                rectangle["pixelHeight"] += 1
            next_active[run] = rectangle
        complete.extend(active.values())
        active = next_active
    complete.extend(active.values())
    return complete


def world_sdf(metadata: dict) -> str:
    width_m, height_m = metadata["widthMeters"], metadata["heightMeters"]
    floor_thickness = metadata["floorThicknessMeters"]
    obstacle_height = metadata["obstacleHeightMeters"]
    origin_x, origin_y, _ = metadata["origin"]
    center_x, center_y = origin_x + width_m / 2, origin_y + height_m / 2
    models = []
    for index, rectangle in enumerate(metadata["obstacles"]):
        models.append(f'''    <model name="map_obstacle_{index}"><static>true</static>
      <pose>{rectangle["worldX"]:.4f} {rectangle["worldY"]:.4f} {obstacle_height / 2:g} 0 0 0</pose><link name="wall">
      <collision name="collision"><geometry><box><size>{rectangle["worldWidth"]:.4f} {rectangle["worldHeight"]:.4f} {obstacle_height:g}</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{rectangle["worldWidth"]:.4f} {rectangle["worldHeight"]:.4f} {obstacle_height:g}</size></box></geometry><material><ambient>0.20 0.28 0.34 1</ambient><diffuse>0.28 0.40 0.48 1</diffuse></material></visual></link></model>''')
    return f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="library">
  <physics name="physics" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><direction>-0.5 0.5 -1</direction></light>
  <model name="floor"><static>true</static><pose>{center_x:.4f} {center_y:.4f} {-floor_thickness / 2:g} 0 0 0</pose><link name="floor_link"><collision name="collision"><geometry><box><size>{width_m:.4f} {height_m:.4f} {floor_thickness:g}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{width_m:.4f} {height_m:.4f} {floor_thickness:g}</size></box></geometry><material><ambient>0.72 0.74 0.72 1</ambient><diffuse>0.82 0.84 0.82 1</diffuse></material></visual></link></model>
{chr(10).join(models)}
</world></sdf>
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--scene", required=True, type=Path)
    args = parser.parse_args()
    config = read_yaml(args.map)
    width, height, maximum, pixels = read_pgm(args.map.parent / str(config["image"]))
    resolution, origin = float(config["resolution"]), config["origin"]
    threshold = int(maximum * (1.0 - float(config.get("occupied_thresh", 0.65))))
    obstacles = []
    for rectangle in occupied_rectangles(width, height, pixels, threshold):
        obstacles.append({**rectangle,
            "worldX": origin[0] + (rectangle["pixelX"] + rectangle["pixelWidth"] / 2) * resolution,
            "worldY": origin[1] + (height - rectangle["pixelY"] - rectangle["pixelHeight"] / 2) * resolution,
            "worldWidth": rectangle["pixelWidth"] * resolution,
            "worldHeight": rectangle["pixelHeight"] * resolution})
    metadata = {"imageWidth": width, "imageHeight": height, "resolution": resolution, "origin": origin,
                "widthMeters": width * resolution, "heightMeters": height * resolution,
                "floorThicknessMeters": FLOOR_THICKNESS_METERS, "obstacleHeightMeters": OBSTACLE_HEIGHT_METERS,
                "obstacles": obstacles}
    args.world.parent.mkdir(parents=True, exist_ok=True)
    args.scene.parent.mkdir(parents=True, exist_ok=True)
    args.world.write_text(world_sdf(metadata), encoding="utf-8")
    args.scene.write_text(json.dumps(metadata, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()

