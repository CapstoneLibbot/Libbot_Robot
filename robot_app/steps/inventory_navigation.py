from __future__ import annotations

import math


def resolve_shelf_endpoints(context, gateway):
    shelf = context.get("current_shelf")
    if not isinstance(shelf, dict) or not shelf.get("shelf_id"):
        raise ValueError("Inventory shelf ID is missing.")
    if gateway is None or gateway.localization_state != "LOCALIZED":
        raise ValueError("Inventory navigation requires localization.")
    pose = gateway._lookup_map_pose()
    if not isinstance(pose, dict):
        raise ValueError("Current map pose is unavailable.")
    try:
        pose_x, pose_y = float(pose["x"]), float(pose["y"])
    except (TypeError, ValueError, KeyError) as exc:
        raise ValueError("Current map pose is invalid.") from exc
    if not math.isfinite(pose_x) or not math.isfinite(pose_y):
        raise ValueError("Current map pose is invalid.")
    def parse_endpoint(data, name):
        if not isinstance(data, dict):
            raise ValueError(f"Inventory scope is missing {name}; refresh the task with the updated backend.")
        try:
            endpoint = {"endpoint_id": data["endpoint_id"], "x": float(data["x"]),
                        "y": float(data["y"]), "theta": float(data["theta"]), "map_id": data["map_id"]}
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Inventory scope has an invalid {name}.") from exc
        if not endpoint["endpoint_id"]:
            raise ValueError(f"Inventory scope has an invalid {name} ID.")
        if not all(math.isfinite(endpoint[k]) for k in ("x", "y", "theta")):
            raise ValueError("Shelf endpoint pose is invalid.")
        return endpoint

    start = parse_endpoint(shelf.get("start_endpoint"), "start_endpoint")
    end_data = shelf.get("end_endpoint")
    if shelf.get("sweep_shelf") is False or end_data is None:
        end = None
    else:
        end = parse_endpoint(end_data, "end_endpoint")
    active_map = getattr(gateway, "_active_map_id", None)
    if not active_map:
        raise ValueError("The active navigation map is unavailable.")
    if not start["map_id"] or str(active_map).lower() != str(start["map_id"]).lower():
        raise ValueError("Shelf endpoints do not belong to the active map.")
    if end is None or str(start["endpoint_id"]).lower() == str(end["endpoint_id"]).lower():
        return start, None
    if (not end["map_id"] or str(start["map_id"]).lower() != str(end["map_id"]).lower()
            or str(active_map).lower() != str(end["map_id"]).lower()):
        raise ValueError("Shelf endpoints must belong to the same active map.")
    a, b = start, end
    dx, dy = b["x"] - a["x"], b["y"] - a["y"]
    length = math.hypot(dx, dy)
    yaw_diff = math.atan2(math.sin(a["theta"] - b["theta"]), math.cos(a["theta"] - b["theta"]))
    forward = dx * math.cos(a["theta"]) + dy * math.sin(a["theta"])
    if not math.isfinite(length) or not math.isfinite(forward) or length < 0.02 or abs(yaw_diff) > 0.1 or abs(forward) > 0.05:
        raise ValueError("Shelf endpoints must be distinct, lateral and have matching orientations.")
    distance = lambda p: math.hypot(p["x"] - pose_x, p["y"] - pose_y)
    return (a, b) if distance(a) <= distance(b) else (b, a)
