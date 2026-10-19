from core.context import RobotContext
from steps.move import MoveStep


def test_move_step_uses_runtime_destination_and_sets_move_status(monkeypatch):
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})
    context.set("destination", {"x": 4.5, "y": 3.0})
    context.set("target_endpoint_id", "SHELF_A3")

    step = MoveStep({
        "endpoint_id": "SHELF_A3",
        "speed_mode": "SLOW_GUIDE",
        "obstacle_policy": "WAIT_RESUME",
    })

    def navigate(_self, goal):
        assert goal["target"]["x"] == 4.5
        assert goal["target"]["y"] == 3.0
        return {
            "move_status": "COMPLETED",
            "final_pose": {"x": 4.5, "y": 3.0, "theta": 0.0},
            "nav2_result_code": "SUCCEEDED",
            "duration_ms": 1,
        }

    monkeypatch.setattr("steps.move.NavigationController.navigate", navigate)
    result = step.execute(context)

    assert result["move_status"] == "COMPLETED"
    assert result["type"] == "MOVE"
    assert result["target"]["x"] == 4.5
    assert result["target"]["theta"] == 0.0
    assert result["nav2_result_code"] == "SUCCEEDED"
    assert context.get("move_status") == "COMPLETED"
    assert context.get("last_move_target")["x"] == 4.5


def test_move_step_fails_when_target_is_missing():
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})

    step = MoveStep({
        "speed_mode": "NORMAL",
        "obstacle_policy": "WAIT_RESUME",
    })

    result = step.execute(context)

    assert result["status"] == "error"
    assert result["move_status"] == "FAILED"
    assert result["nav2_result_code"] == "INVALID_INPUT"
    assert context.get("move_status") == "FAILED"


def test_move_step_rejects_endpoint_id_without_resolved_pose():
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})

    step = MoveStep({
        "endpoint_id": "SHELF_A1",
        "speed_mode": "NORMAL",
        "obstacle_policy": "REROUTE",
        "timeout_sec": 30,
    })

    result = step.execute(context)

    assert result["status"] == "error"
    assert result["move_status"] == "FAILED"
    assert result["nav2_result_code"] == "INVALID_INPUT"
    assert "requires x and y" in result["message"]


def test_move_step_rejects_path_id_without_route_resolver():
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})

    step = MoveStep({
        "path_id": "PATROL_ROUTE_1",
        "speed_mode": "PATROL",
        "obstacle_policy": "DETECT_AND_WAIT",
        "scan_while_moving": True,
    })

    result = step.execute(context)

    assert result["status"] == "error"
    assert result["move_status"] == "FAILED"
    assert result["nav2_result_code"] == "INVALID_INPUT"
    assert "route resolver" in result["message"]
