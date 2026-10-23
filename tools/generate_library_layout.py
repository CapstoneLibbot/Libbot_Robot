#!/usr/bin/env python3
"""Generate the Gazebo, Nav2, and FE assets from one library layout JSON."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional


BOOK_COLORS = ["#3D7794", "#B95D46", "#D7A84D", "#66855C", "#765D8B", "#D07A45"]
WOOD = "#8B5A32"
WOOD_DARK = "#694326"
WOOD_LIGHT = "#B78049"
WALL_COLOR = "#E7DFCF"
FLOOR_COLOR = "#C8B99F"
CHAIR_COLOR = "#66855C"
PORCELAIN = "#F4F5F3"
WATER_COLOR = "#6FA9B7"


def box_part(name: str, size: list[float], offset: list[float], color: str) -> dict:
    return {"name": name, "size": size, "offset": offset, "color": color}


def furniture_parts(item: dict) -> list[dict]:
    width, depth, height = item["size"]
    kind = item["type"]

    if kind == "bookshelf":
        parts = [
            box_part("left_side", [width, 0.045, height], [0, -depth / 2 + 0.0225, height / 2], WOOD_DARK),
            box_part("right_side", [width, 0.045, height], [0, depth / 2 - 0.0225, height / 2], WOOD_DARK),
            box_part("back", [0.04, depth, height], [width / 2 - 0.02, 0, height / 2], WOOD),
        ]
        levels = [height * fraction for fraction in (0.14, 0.38, 0.62, 0.82)]
        book_count = max(7, min(18, round(depth / 0.52)))
        book_pitch = (depth - 0.12) / max(book_count - 1, 1)
        book_depth = min(0.34, book_pitch * 0.78)
        book_height = min(0.40, max(0.18, height * 0.14))
        for shelf_index, z in enumerate(levels):
            parts.append(box_part(f"shelf_{shelf_index + 1}", [width, depth, 0.04], [0, 0, z], WOOD_LIGHT))
            book_bottom = z + 0.02
            for book_index in range(book_count):
                y = -depth / 2 + 0.06 + book_index * book_pitch
                current_book_height = book_height * (0.86 if (book_index + shelf_index) % 3 == 0 else 1)
                parts.append(box_part(
                    f"book_{shelf_index + 1}_{book_index + 1}",
                    [width * 0.68, book_depth, current_book_height],
                    [-width * 0.08, y, book_bottom + current_book_height / 2],
                    BOOK_COLORS[(book_index + shelf_index) % len(BOOK_COLORS)],
                ))
        return parts

    if kind == "counter":
        return [
            box_part("cabinet", [width, depth * 0.9, height * 0.88], [0, 0, height * 0.44], WOOD),
            box_part("countertop", [width + 0.10, depth + 0.08, 0.08], [0, 0, height * 0.94], WOOD_DARK),
            box_part("service_panel", [width * 0.68, 0.035, 0.22], [0, -depth * 0.47, height * 0.73], WOOD_LIGHT),
            box_part("terminal", [0.22, 0.16, 0.035], [0, depth * 0.16, height + 0.035], "#344B54"),
        ]

    if kind == "reading_table":
        parts = [box_part("tabletop", [width, depth, 0.07], [0, 0, height - 0.035], WOOD_LIGHT)]
        leg_height = height - 0.07
        for x_sign in (-1, 1):
            for y_sign in (-1, 1):
                parts.append(box_part(
                    f"leg_{x_sign}_{y_sign}", [0.065, 0.065, leg_height],
                    [x_sign * (width / 2 - 0.05), y_sign * (depth / 2 - 0.05), leg_height / 2], WOOD_DARK,
                ))
        return parts

    if kind == "chair":
        seat_height = 0.44
        leg_height = seat_height - 0.07
        parts = [
            box_part("seat", [width, depth, 0.07], [0, 0, seat_height], WOOD_LIGHT),
            box_part("back", [0.07, depth, height - seat_height], [-width / 2 + 0.035, 0, (height + seat_height) / 2], CHAIR_COLOR),
        ]
        for x_sign in (-1, 1):
            for y_sign in (-1, 1):
                parts.append(box_part(
                    f"leg_{x_sign}_{y_sign}", [0.05, 0.05, leg_height],
                    [x_sign * (width / 2 - 0.04), y_sign * (depth / 2 - 0.04), leg_height / 2], WOOD_DARK,
                ))
        return parts

    if kind == "toilet_fixture":
        return [
            box_part("pedestal", [width * 0.66, depth * 0.78, height * 0.35], [width * 0.12, 0, height * 0.175], PORCELAIN),
            box_part("bowl", [width, depth, height * 0.28], [0, 0, height * 0.43], "#FFFFFF"),
            box_part("cistern", [width * 0.55, depth * 0.72, height * 0.42], [-width * 0.22, 0, height * 0.79], PORCELAIN),
        ]

    if kind == "sink":
        return [
            box_part("pedestal", [width * 0.32, depth * 0.36, height * 0.72], [width * 0.2, 0, height * 0.36], PORCELAIN),
            box_part("basin", [width, depth, height * 0.18], [0, 0, height * 0.82], "#FFFFFF"),
            box_part("water", [width * 0.52, depth * 0.48, height * 0.03], [0, 0, height * 0.88], WATER_COLOR),
        ]

    raise ValueError(f"Unsupported furniture type: {kind}")


def scene_objects(layout: dict) -> tuple[list[dict], list[dict]]:
    room = layout["room"]
    half_x = room["widthX"] / 2
    half_y = room["lengthY"] / 2
    wall_t = room["wallThickness"]
    wall_h = room["wallHeight"]
    door_w = room["doorWidth"]
    segments = [
        ("wall_west", -half_x, 0, [wall_t, room["lengthY"] + wall_t, wall_h]),
        ("wall_east", half_x, 0, [wall_t, room["lengthY"] + wall_t, wall_h]),
        ("wall_north", 0, half_y, [room["widthX"], wall_t, wall_h]),
    ]
    south_width = (room["widthX"] - door_w) / 2
    segments.extend([
        ("wall_south_west", -(door_w / 2 + south_width / 2), -half_y, [south_width, wall_t, wall_h]),
        ("wall_south_east", door_w / 2 + south_width / 2, -half_y, [south_width, wall_t, wall_h]),
    ])

    components: list[dict] = []
    collision_shapes: list[dict] = []
    for name, x, y, size in segments:
        components.append({
            "name": name, "category": "wall", "label": "Library wall",
            "worldX": x, "worldY": y, "worldZ": wall_h / 2,
            "size": size, "color": WALL_COLOR,
        })
        collision_shapes.append({"name": name, "x": x, "y": y, "size": size[:2]})

    for room in layout.get("rooms", []):
        room_width, room_length = room["size"]
        room_wall_data = room_wall_segments(room)
        components.append({
            "name": f"{room['id']}_floor",
            "category": "room_floor",
            "label": room["label"],
            "worldX": room["x"],
            "worldY": room["y"],
            "worldZ": 0.004,
            "size": [room_width, room_length, 0.008],
            "color": room["color"],
        })
        for name, x, y, size in room_wall_data:
            components.append({
                "name": name,
                "category": "partition",
                "label": room["label"],
                "worldX": x,
                "worldY": y,
                "worldZ": size[2] / 2,
                "size": size,
                "color": WALL_COLOR,
            })
            collision_shapes.append({"name": name, "x": x, "y": y, "size": size[:2]})

    for item in layout["furniture"]:
        width, depth, height = item["size"]
        x, y = item["x"], item["y"]
        collision_shapes.append({"name": item["id"], "x": x, "y": y, "size": [width, depth]})
        for part in furniture_parts(item):
            offset_x, offset_y, offset_z = part["offset"]
            components.append({
                "name": f"{item['id']}_{part['name']}",
                "category": item["type"],
                "label": item["label"],
                "worldX": x + offset_x,
                "worldY": y + offset_y,
                "worldZ": offset_z,
                "size": part["size"],
                "color": part["color"],
            })
    return components, collision_shapes


def room_wall_segments(room: dict) -> list[tuple[str, float, float, list[float]]]:
    width, length = room["size"]
    center_x, center_y = room["x"], room["y"]
    thickness = room.get("wallThickness", 0.18)
    height = room.get("wallHeight", 2.5)
    door_side = str(room.get("doorSide", "")).lower()
    doorway = float(room.get("doorwayWidth", 0.0)) if door_side else 0.0
    result: list[tuple[str, float, float, list[float]]] = []

    for side in ("north", "south", "east", "west"):
        horizontal = side in ("north", "south")
        span = width if horizontal else length
        along_center = center_x if horizontal else center_y
        cross_center = center_y + (length / 2 if side == "north" else -length / 2) if side in ("north", "south") else center_x + (width / 2 if side == "east" else -width / 2)
        axis_size = span
        wall_size = [axis_size, thickness, height] if horizontal else [thickness, axis_size, height]

        if side != door_side or doorway <= 0:
            result.append((f"{room['id']}_{side}_wall", along_center, cross_center, wall_size))
            continue

        gap = min(doorway, span - 2 * thickness)
        segment = (span - gap) / 2
        if segment <= 0:
            continue
        for direction, suffix in ((-1, "a"), (1, "b")):
            offset = direction * (gap / 2 + segment / 2)
            x = along_center + offset if horizontal else cross_center
            y = cross_center if horizontal else along_center + offset
            size = [segment, thickness, height] if horizontal else [thickness, segment, height]
            result.append((f"{room['id']}_{side}_wall_{suffix}", x, y, size))
    return result


def validate_layout(layout: dict, collision_shapes: list[dict]) -> None:
    room = layout["room"]
    width, length = room["widthX"], room["lengthY"]
    resolution = room["resolution"]
    if abs(width / resolution - round(width / resolution)) > 1e-8:
        raise ValueError("Room width must be an integer multiple of map resolution")
    if abs(length / resolution - round(length / resolution)) > 1e-8:
        raise ValueError("Room length must be an integer multiple of map resolution")
    if room["doorWidth"] <= 0 or room["doorWidth"] >= width:
        raise ValueError("South doorway must be wider than zero and narrower than the room")

    half_x, half_y = width / 2, length / 2
    for item in layout["furniture"]:
        sx, sy, _ = item["size"]
        if item["x"] - sx / 2 < -half_x or item["x"] + sx / 2 > half_x:
            raise ValueError(f"{item['id']} extends outside the room in X")
        if item["y"] - sy / 2 < -half_y or item["y"] + sy / 2 > half_y:
            raise ValueError(f"{item['id']} extends outside the room in Y")

    for room in layout.get("rooms", []):
        room_width, room_length = room["size"]
        if room_width <= 0 or room_length <= 0:
            raise ValueError(f"{room['id']} must have positive dimensions")
        if room["doorSide"] not in {"north", "south", "east", "west"}:
            raise ValueError(f"{room['id']} must have a valid doorSide")
        if room["doorwayWidth"] <= 0 or room["doorwayWidth"] >= min(room_width, room_length):
            raise ValueError(f"{room['id']} must have a positive doorway narrower than the room")
        if (room["x"] - room_width / 2 < -width / 2
                or room["x"] + room_width / 2 > width / 2
                or room["y"] - room_length / 2 < -length / 2
                or room["y"] + room_length / 2 > length / 2):
            raise ValueError(f"{room['id']} extends outside the room bounds")

    spawn = layout["robotSpawn"]
    for shape in collision_shapes:
        sx, sy = shape["size"]
        if abs(spawn["x"] - shape["x"]) < sx / 2 and abs(spawn["y"] - shape["y"]) < sy / 2:
            raise ValueError(f"Robot spawn overlaps {shape['name']}")

    target = layout.get("simulationBookTarget")
    if target:
        shelf = next((item for item in layout["furniture"] if item["id"] == target["shelfId"]), None)
        if shelf is None or shelf["type"] != "bookshelf":
            raise ValueError("simulationBookTarget must point to a configured bookshelf")
        if not (-half_x <= target["x"] <= half_x and -half_y <= target["y"] <= half_y):
            raise ValueError("simulationBookTarget must be within the library map")
        for shape in collision_shapes:
            dx = max(abs(target["x"] - shape["x"]) - shape["size"][0] / 2, 0)
            dy = max(abs(target["y"] - shape["y"]) - shape["size"][1] / 2, 0)
            if math.hypot(dx, dy) < 0.6:
                raise ValueError(f"simulationBookTarget is too close to {shape['name']}")


def xml_color(hex_color: str) -> tuple[str, str]:
    color = hex_color.lstrip("#")
    rgb = [int(color[index:index + 2], 16) / 255 for index in (0, 2, 4)]
    diffuse = " ".join(f"{channel:.3f}" for channel in rgb) + " 1"
    ambient = " ".join(f"{channel * 0.65:.3f}" for channel in rgb) + " 1"
    return ambient, diffuse


def add_box_geometry(parent: ET.Element, size: list[float]) -> None:
    geometry = ET.SubElement(parent, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = " ".join(f"{value:g}" for value in size)


def add_material(parent: ET.Element, color: str) -> None:
    material = ET.SubElement(parent, "material")
    ambient, diffuse = xml_color(color)
    ET.SubElement(material, "ambient").text = ambient
    ET.SubElement(material, "diffuse").text = diffuse


def world_sdf(layout: dict) -> str:
    room = layout["room"]
    width, length = room["widthX"], room["lengthY"]
    root = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(root, "world", {"name": layout["worldName"]})
    physics = ET.SubElement(world, "physics", {"name": "physics", "type": "dart"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    for filename, name in [
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ]:
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    sensor = ET.SubElement(world, "plugin", {
        "filename": "gz-sim-sensors-system", "name": "gz::sim::systems::Sensors",
    })
    ET.SubElement(sensor, "render_engine").text = "ogre2"
    light = ET.SubElement(world, "light", {"type": "directional", "name": "sun"})
    ET.SubElement(light, "pose").text = "0 0 8 0 0 0"
    ET.SubElement(light, "direction").text = "-0.5 0.4 -1"

    floor = ET.SubElement(world, "model", {"name": "floor"})
    ET.SubElement(floor, "static").text = "true"
    ET.SubElement(floor, "pose").text = f"0 0 {-room['floorThickness'] / 2:g} 0 0 0"
    floor_link = ET.SubElement(floor, "link", {"name": "floor_link"})
    collision = ET.SubElement(floor_link, "collision", {"name": "collision"})
    add_box_geometry(collision, [width, length, room["floorThickness"]])
    visual = ET.SubElement(floor_link, "visual", {"name": "visual"})
    add_box_geometry(visual, [width, length, room["floorThickness"]])
    add_material(visual, FLOOR_COLOR)

    for area in layout.get("rooms", []):
        room_width, room_length = area["size"]
        append_static_model(
            world, f"{area['id']}_floor", area["x"], area["y"], 0,
            [box_part("floor", [room_width, room_length, 0.008], [0, 0, 0.004], area["color"])],
            None,
        )
        for name, x, y, size in room_wall_segments(area):
            append_static_model(
                world, name, x, y, 0,
                [box_part("wall", size, [0, 0, size[2] / 2], WALL_COLOR)],
                size,
            )

    wall_data = [
        ("wall_west", -width / 2, 0, [room["wallThickness"], length + room["wallThickness"], room["wallHeight"]]),
        ("wall_east", width / 2, 0, [room["wallThickness"], length + room["wallThickness"], room["wallHeight"]]),
        ("wall_north", 0, length / 2, [width, room["wallThickness"], room["wallHeight"]]),
    ]
    south_width = (width - room["doorWidth"]) / 2
    wall_data.extend([
        ("wall_south_west", -(room["doorWidth"] / 2 + south_width / 2), -length / 2, [south_width, room["wallThickness"], room["wallHeight"]]),
        ("wall_south_east", room["doorWidth"] / 2 + south_width / 2, -length / 2, [south_width, room["wallThickness"], room["wallHeight"]]),
    ])
    for name, x, y, size in wall_data:
        append_static_model(world, name, x, y, 0, [
            {"name": "wall", "size": size, "offset": [0, 0, size[2] / 2], "color": WALL_COLOR}
        ], size)

    for item in layout["furniture"]:
        append_static_model(
            world, item["id"], item["x"], item["y"], 0,
            furniture_parts(item), item["size"],
        )

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def append_static_model(
    world: ET.Element,
    name: str,
    x: float,
    y: float,
    yaw: float,
    parts: list[dict],
    collision_size: Optional[list[float]],
) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x:g} {y:g} 0 0 0 {yaw:g}"
    link = ET.SubElement(model, "link", {"name": "link"})
    if collision_size is not None:
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        ET.SubElement(collision, "pose").text = f"0 0 {collision_size[2] / 2:g} 0 0 0"
        add_box_geometry(collision, collision_size)
    for part in parts:
        visual = ET.SubElement(link, "visual", {"name": part["name"]})
        ET.SubElement(visual, "pose").text = " ".join(f"{value:g}" for value in [*part["offset"], 0, 0, 0])
        add_box_geometry(visual, part["size"])
        add_material(visual, part["color"])


def occupancy_pgm(layout: dict, collision_shapes: list[dict]) -> bytes:
    room = layout["room"]
    resolution = room["resolution"]
    width = round(room["widthX"] / resolution)
    height = round(room["lengthY"] / resolution)
    origin_x, origin_y = -room["widthX"] / 2, -room["lengthY"] / 2
    pixels = bytearray([254] * (width * height))
    image_top_y = origin_y + room["lengthY"]
    for shape in collision_shapes:
        size_x, size_y = shape["size"]
        shape_x_min, shape_x_max = shape["x"] - size_x / 2, shape["x"] + size_x / 2
        shape_y_min, shape_y_max = shape["y"] - size_y / 2, shape["y"] + size_y / 2
        first_column = max(0, math.floor((shape_x_min - origin_x) / resolution))
        past_last_column = min(width, math.ceil((shape_x_max - origin_x) / resolution))
        first_row = max(0, math.floor((image_top_y - shape_y_max) / resolution))
        past_last_row = min(height, math.ceil((image_top_y - shape_y_min) / resolution))
        row_data = bytes(past_last_column - first_column)
        for row in range(first_row, past_last_row):
            row_start = row * width + first_column
            pixels[row_start:row * width + past_last_column] = row_data[:past_last_column - first_column]
    header = f"P5\n# Generated from library_layout.json\n{width} {height}\n255\n".encode("ascii")
    return header + bytes(pixels)


def generate(layout_path: Path, world_path: Path, map_dir: Path, scene_path: Path) -> None:
    layout = json.loads(layout_path.read_text(encoding="utf-8"))
    components, collision_shapes = scene_objects(layout)
    validate_layout(layout, collision_shapes)
    room = layout["room"]
    map_width = round(room["widthX"] / room["resolution"])
    map_height = round(room["lengthY"] / room["resolution"])
    origin = [-room["widthX"] / 2, -room["lengthY"] / 2, 0]
    scene = {
        "origin": origin,
        "widthMeters": room["widthX"],
        "heightMeters": room["lengthY"],
        "floorThicknessMeters": room["floorThickness"],
        "wallHeightMeters": room["wallHeight"],
        "imageWidth": map_width,
        "imageHeight": map_height,
        "resolution": room["resolution"],
        "robotSpawn": layout["robotSpawn"],
        "simulationBookTarget": layout.get("simulationBookTarget"),
        "areas": [
            {
                "id": room["id"], "type": room["type"], "label": room["label"],
                "x": room["x"], "y": room["y"], "size": room["size"], "color": room["color"],
            }
            for room in layout.get("rooms", [])
        ],
        "bookshelfCount": sum(item["type"] == "bookshelf" for item in layout["furniture"]),
        "components": components,
    }
    world_path.parent.mkdir(parents=True, exist_ok=True)
    map_dir.mkdir(parents=True, exist_ok=True)
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(world_sdf(layout), encoding="utf-8")
    (map_dir / "library_sim.pgm").write_bytes(occupancy_pgm(layout, collision_shapes))
    map_yaml = (
        "image: library_sim.pgm\n"
        "mode: trinary\n"
        f"resolution: {room['resolution']:.3f}\n"
        f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    (map_dir / "library_sim.yaml").write_text(map_yaml, encoding="utf-8")
    scene_path.write_text(json.dumps(scene, indent=2), encoding="utf-8")
    (scene_path.parent / "library_sim.pgm").write_bytes((map_dir / "library_sim.pgm").read_bytes())
    (scene_path.parent / "library_sim.yaml").write_text(map_yaml, encoding="utf-8")


def main() -> None:
    libot_dir = Path(__file__).resolve().parents[1]
    project_root = libot_dir.parent.parent
    description_dir = libot_dir / "ros2_ws" / "src" / "libot_description"
    parser = argparse.ArgumentParser()
    parser.add_argument("--layout", type=Path, default=description_dir / "config" / "library_layout.json")
    parser.add_argument("--world", type=Path, default=description_dir / "worlds" / "library.sdf")
    parser.add_argument("--map-dir", type=Path, default=libot_dir / "maps")
    parser.add_argument("--scene", type=Path, default=project_root / "FE" / "public" / "maps" / "library-map.json")
    args = parser.parse_args()
    generate(args.layout, args.world, args.map_dir, args.scene)


if __name__ == "__main__":
    main()
