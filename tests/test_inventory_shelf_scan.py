import pytest

from core.context import RobotContext
from core.workflow_engine import WorkflowEngine
from steps.inventory_navigation import resolve_shelf_endpoints
from steps.scan import ScanStep
from steps.move import MoveStep
from hardware.shelf_motion import _bounded_correction, _shelf_velocity


class Gateway:
    localization_state = "LOCALIZED"
    _active_map_id = "map-1"

    def __init__(self, x=0, y=0):
        self.pose = {"x": x, "y": y, "yaw": 0.0}

    def _lookup_map_pose(self):
        return dict(self.pose)

    def _publish_stop(self):
        pass

    def navigation_zone_blocking_action(self, *args):
        return None


@pytest.mark.parametrize("error", [-0.045, 0.045])
def test_shelf_small_lateral_error_has_bounded_motion(error):
    velocity = _bounded_correction(error, 0.04, 0.7, 0.04, 0.5)
    assert abs(velocity) == pytest.approx(0.04)
    assert velocity * error > 0
    assert _bounded_correction(error / 2, 0.04, 0.7, 0.04, 0.5) == 0


def test_previous_alignment_error_is_within_new_lateral_tolerance():
    assert _bounded_correction(-0.034, 0.04, 0.7, 0.04, 0.5) == 0


def test_shelf_heading_correction_stops_in_tolerance_and_respects_limit():
    assert _bounded_correction(0.039, 0.04, 0.8, 0.08, 0.15) == 0
    assert _bounded_correction(-0.05, 0.04, 0.8, 0.08, 0.15) == pytest.approx(-0.08)
    assert _bounded_correction(0.4, 0.04, 0.8, 0.08, 0.15) == pytest.approx(0.15)


def test_shelf_motion_is_faster_and_corrects_heading_separately():
    assert _shelf_velocity(0.15, 0.0, 0.04, 0.15, 0.04, 0.5, 0.08) == pytest.approx((0.18, 0))
    assert _shelf_velocity(0.15, 0.10, 0.04, 0.04, 0.04, 0.5, 0.08) == pytest.approx((0, 0.08))
    assert _shelf_velocity(0.03, 0.0, 0.04, 0.15, 0.04, 0.5, 0.08) == (0, 0)


def test_inventory_alignment_failure_reports_nav_result_and_actual_goal(monkeypatch):
    context = RobotContext()
    gateway = Gateway()
    start = {"endpoint_id": "a", "x": 0.0, "y": 0.0, "theta": 0.0, "map_id": "map-1"}
    end = {"endpoint_id": "b", "x": 0.0, "y": 0.2, "theta": 0.0, "map_id": "map-1"}
    context.set("current_shelf", {"shelf_id": "shelf-1", "start_endpoint": start, "end_endpoint": end})
    monkeypatch.setattr("steps.move.NavigationController.navigate", lambda self, goal: {
        "move_status": "COMPLETED", "nav2_result_code": "SUCCEEDED"})
    def fail_alignment(self, target, context):
        raise RuntimeError("Shelf scan movement timed out")
    monkeypatch.setattr("hardware.shelf_motion.ShelfMotionController.move_to", fail_alignment)
    step = MoveStep({"_inventory_shelf_scan": True, "_runtime_gateway": gateway})
    result = step.execute(context)
    assert result["move_status"] == "FAILED"
    assert result["error_code"] == "SHELF_ALIGNMENT_FAILED"
    assert result["nav2_result_code"] == "SUCCEEDED"
    assert result["target"] == start
    assert result["current_pose"] == gateway.pose


def test_selects_nearest_endpoint_from_inventory_scope(monkeypatch):
    context = RobotContext()
    shelf = {
        "shelf_id": "shelf-1",
        "start_endpoint": {"endpoint_id": "a", "x": 1, "y": 0, "theta": 0, "map_id": "map-1"},
        "end_endpoint": {"endpoint_id": "b", "x": 1, "y": 1, "theta": 0, "map_id": "map-1"},
    }
    context.set("current_shelf", shelf)
    def no_lookup(*args, **kwargs):
        pytest.fail("Pi must not fetch shelf or endpoint details")
    monkeypatch.setattr("steps.process_data.ProcessDataStep._json_request", no_lookup)
    start, end = resolve_shelf_endpoints(context, Gateway(1, 0.9))
    assert (start["endpoint_id"], end["endpoint_id"]) == ("b", "a")
    start, _ = resolve_shelf_endpoints(context, Gateway(1, 0.5))
    assert start["endpoint_id"] == "a"
    shelf["end_endpoint"]["map_id"] = "other"
    with pytest.raises(ValueError, match="same map"):
        resolve_shelf_endpoints(context, Gateway())


def test_old_inventory_scope_without_two_endpoints_fails():
    context = RobotContext()
    context.set("current_shelf", {"shelf_id": "shelf-1", "endpoint": {"x": 1, "y": 1}})
    with pytest.raises(ValueError, match="missing start_endpoint"):
        resolve_shelf_endpoints(context, Gateway())


def inventory_workflow():
    def step(id, kind, name, params, transitions):
        return {"id": id, "type": kind, "name": name, "params": params,
                "output_mapping": {}, "transitions": transitions, "policy": {}}
    return [
        step("I40", "MOVE", "MOVE_TO_INVENTORY_SHELF", {"_inventory_shelf_scan": True}, {"on_success": "I50", "on_error": "ERR"}),
        {**step("I50", "SCAN", "SCAN_QR_ON_BOTH_SHELF_LEVELS", {"scan_type": "QR", "target_type": "BOOK", "continuous": True},
                {"on_success": "I60", "on_error": "ERR"}),
         "output_mapping": {"scanned_items": "scanned_items", "scan_status": "scan_status"}},
        step("I60", "PROCESS_DATA", "VERIFY_SCANNED_BOOKS", {"action": "VERIFY_BOOK"}, {"on_success": "DONE", "on_error": "ERR"}),
        step("DONE", "END", "DONE", {"result_status": "SUCCEEDED"}, {}),
        step("ERR", "END", "ERR", {"result_status": "FAILED"}, {}),
    ]


def run_inventory(monkeypatch, distance, scans, blocked=False):
    gateway = Gateway()
    engine = WorkflowEngine({"runtime": {"backend_url": "http://backend"}}, None, gateway)
    start = {"endpoint_id": "a", "x": 0.0, "y": 0.0, "theta": 0.0, "map_id": "map-1"}
    end = {"endpoint_id": "b", "x": 0.0, "y": distance, "theta": 0.0, "map_id": "map-1"}
    engine.context.update_variables({"runtime_id": "sim", "scan_queue": scans,
        "current_shelf": {"shelf_id": "shelf-1", "start_endpoint": start, "end_endpoint": end}})
    monkeypatch.setattr("steps.move.NavigationController.navigate", lambda self, goal: {
        "move_status": "COMPLETED", "final_pose": start})
    targets = []
    def move_to(self, target, context):
        if blocked and target["y"]:
            raise RuntimeError("blocked")
        targets.append(dict(target))
        gateway.pose.update(target)
    monkeypatch.setattr("hardware.shelf_motion.ShelfMotionController.move_to", move_to)
    monkeypatch.setattr("hardware.shelf_motion.ShelfMotionController.wait_stopped", lambda self, context: None)
    verified = []
    def verify(self, context):
        verified.append(context.get("scanned_items"))
        return {"status": "processed", "type": "PROCESS_DATA"}
    monkeypatch.setattr("steps.process_data.ProcessDataStep.execute", verify)
    result = engine.run_steps(inventory_workflow())
    return result, engine.context, targets, verified


@pytest.mark.parametrize("distance,count", [(0.1, 2), (0.2, 3), (0.45, 4), (5.0, 35)])
def test_expanded_workflow_moves_scans_and_evaluates(monkeypatch, distance, count):
    scan = [{"qr_code": "A", "level_number": 1}, {"qr_code": "A", "level_number": 2}]
    result, context, targets, verified = run_inventory(monkeypatch, distance, [scan] * count)
    assert result["status"] == "ok", result
    steps = [item["step_type"] for item in result["results"]]
    assert steps == ["MOVE", "SCAN"] + ["MOVE", "SCAN", "EVALUATE"] * (count - 1) + ["PROCESS_DATA", "END"]
    assert len(verified) == 1 and len(verified[0]) == 2
    assert targets[-1]["y"] == pytest.approx(distance)
    assert all(target["y"] - previous <= 0.150001 for previous, target in zip([0] + [p["y"] for p in targets], targets))
    assert context.get("inventory_scan_complete") is True


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("distance,expected", [
    (0.35, [0, 0.15, 0.30, 0.35]),
    (0.40, [0, 0.15, 0.30, 0.40]),
    (0.505, [0, 0.15, 0.30, 0.45, 0.505]),
])
def test_scan_targets_use_15cm_segments_and_finish_at_endpoint(monkeypatch, direction, distance, expected):
    scan = [{"qr_code": "A", "level_number": 1}]
    result, _, targets, _ = run_inventory(monkeypatch, direction * distance, [scan] * len(expected))
    assert result["status"] == "ok", result
    assert [target["y"] for target in targets] == pytest.approx([direction * value for value in expected])
    segments = [item["result"] for item in result["results"] if item["step"] == "MOVE_ALONG_INVENTORY_SHELF"]
    assert [item["actual_displacement_m"] for item in segments] == pytest.approx(
        [b - a for a, b in zip(expected, expected[1:])])


def test_empty_shelf_does_not_reach_verify(monkeypatch):
    result, _, _, verified = run_inventory(monkeypatch, 0.2, [[], [], []])
    assert result["status"] == "error"
    assert not verified
    assert "No books" in result["error"]


def test_blocked_move_does_not_reach_verify(monkeypatch):
    result, _, _, verified = run_inventory(monkeypatch, 0.2, [[{"qr_code": "A", "level_number": 1}]], blocked=True)
    assert result["status"] == "error"
    assert not verified
    assert "blocked" in result["error"]


def test_scanner_error_does_not_reach_verify(monkeypatch):
    monkeypatch.setattr(ScanStep, "_scan_once", lambda self, context: {
        "status": "error", "type": "SCAN", "scan_status": "FAILED", "message": "scanner failed"})
    result, _, _, verified = run_inventory(monkeypatch, 0.2, [])
    assert result["status"] == "error"
    assert not verified
    assert "scanner failed" in result["error"]


def test_expansion_keeps_definition_and_avoids_id_collision():
    steps = inventory_workflow()
    steps.append({"id": "__inventory_scan_move", "type": "END", "name": "other"})
    expanded = WorkflowEngine._expand_inventory_workflow(steps)
    assert steps[1]["transitions"]["on_success"] == "I60"
    assert len({item["id"] for item in expanded}) == len(expanded)
    assert expanded[1]["transitions"]["on_success"] != "I60"


def test_scan_does_not_move_robot():
    context = RobotContext()
    context.update_variables({"runtime_id": "sim", "scan_queue": [[{"qr_code": "A", "level_number": 1}]],
                              "inventory_scanned_items": [], "inventory_scan_complete": False})
    step = ScanStep({"_inventory_shelf_scan": True, "scan_type": "QR", "target_type": "BOOK"})
    assert step.execute(context)["scanned_items"][0]["qr_code"] == "A"
    assert not hasattr(step, "inventory_navigation")


def test_multiple_shelves_clear_previous_scan_and_generate_report(monkeypatch):
    gateway = Gateway()
    engine = WorkflowEngine({"runtime": {"backend_url": "http://backend"}}, None, gateway)
    def shelf_data(name, y):
        return {"shelf_id": name,
            "start_endpoint": {"endpoint_id": name + "1", "x": 0.0, "y": y, "theta": 0.0, "map_id": "map-1"},
            "end_endpoint": {"endpoint_id": name + "2", "x": 0.0, "y": y + 0.2, "theta": 0.0, "map_id": "map-1"}}
    engine.context.update_variables({"runtime_id": "sim", "current_shelf": shelf_data("A", 0),
        "scan_queue": [[{"qr_code": code, "level_number": 1}] for code in ("A", "A", "A", "B", "B", "B")]})
    monkeypatch.setattr("steps.move.NavigationController.navigate", lambda self, goal: {
        "move_status": "COMPLETED", "final_pose": goal["target"]})
    monkeypatch.setattr("hardware.shelf_motion.ShelfMotionController.move_to",
                        lambda self, target, context: gateway.pose.update(target))
    monkeypatch.setattr("hardware.shelf_motion.ShelfMotionController.wait_stopped", lambda self, context: None)
    verified, reports = [], []
    def process(self, context):
        if self.operation == "VERIFY_BOOK":
            verified.append([item["qr_code"] for item in context.get("scanned_items")])
            context.set("has_next", len(verified) == 1)
            if len(verified) == 1:
                context.set("current_shelf", shelf_data("B", 0.2))
        elif self.operation == "GENERATE_REPORT":
            reports.append(True)
        return {"status": "processed", "type": "PROCESS_DATA"}
    monkeypatch.setattr("steps.process_data.ProcessDataStep.execute", process)
    steps = inventory_workflow()
    steps[2]["transitions"]["on_success"] = "NEXT"
    steps.extend([
        {"id": "NEXT", "type": "EVALUATE", "name": "NEXT", "params": {"left": "${has_next}", "operator": "EQUAL", "right": True},
         "output_mapping": {}, "transitions": {"on_true": "I40", "on_false": "REPORT", "on_error": "ERR"}, "policy": {}},
        {"id": "REPORT", "type": "PROCESS_DATA", "name": "GENERATE_REPORT", "params": {"action": "GENERATE_REPORT"},
         "output_mapping": {}, "transitions": {"on_success": "DONE", "on_error": "ERR"}, "policy": {}},
    ])
    result = engine.run_steps(steps)
    assert result["status"] == "ok", result
    assert verified == [["A"], ["B"]]
    assert reports == [True]
