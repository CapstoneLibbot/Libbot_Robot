from core.context import RobotContext
from steps.move import MoveStep


def test_move_step_uses_runtime_destination_and_sets_move_status():
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})
    context.set("destination", {"x": 4.5, "y": 3.0})
    context.set("target_endpoint_id", "SHELF_A3")

    step = MoveStep({
        "endpoint_id": "SHELF_A3",
        "speed_mode": "SLOW_GUIDE",
        "obstacle_policy": "WAIT_RESUME",
    })

    result = step.execute(context)

    assert result["move_status"] == "COMPLETED"
    assert result["type"] == "MOVE"
    assert result["target"]["x"] == 4.5
    assert result["target"]["theta"] == 0.0
    assert result["nav2_result_code"] == "SIMULATED_GOAL_REACHED"
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


def test_move_step_accepts_endpoint_id_goal():
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})

    step = MoveStep({
        "endpoint_id": "SHELF_A1",
        "speed_mode": "NORMAL",
        "obstacle_policy": "REROUTE",
        "timeout_sec": 30,
    })

    result = step.execute(context)

    assert result["status"] == "ok"
    assert result["move_status"] == "COMPLETED"
    assert result["target"] == {"endpoint_id": "SHELF_A1"}
    assert result["timeout_sec"] == 30
    assert result["nav2_result_code"] == "SIMULATED_ENDPOINT_ACCEPTED"
    assert context.get("target_endpoint_id") == "SHELF_A1"


def test_move_step_accepts_path_id_goal():
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})

    step = MoveStep({
        "path_id": "PATROL_ROUTE_1",
        "speed_mode": "PATROL",
        "obstacle_policy": "DETECT_AND_WAIT",
        "scan_while_moving": True,
    })

    result = step.execute(context)

    assert result["status"] == "ok"
    assert result["move_status"] == "COMPLETED"
    assert result["target"] == {"path_id": "PATROL_ROUTE_1"}
    assert result["scan_while_moving"] is True
    assert result["nav2_result_code"] == "SIMULATED_PATH_COMPLETED"
