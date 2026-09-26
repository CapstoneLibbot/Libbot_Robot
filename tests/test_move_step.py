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

    assert result["move_status"] == "completed"
    assert result["type"] == "MOVE"
    assert result["target"]["x"] == 4.5
    assert context.get("move_status") == "completed"
    assert context.get("last_move_target")["x"] == 4.5


def test_move_step_fails_when_target_is_missing():
    context = RobotContext()
    context.set("current_pose", {"x": 0.0, "y": 0.0, "theta": 0.0})

    step = MoveStep({
        "endpoint_id": "SHELF_A3",
        "speed_mode": "NORMAL",
        "obstacle_policy": "WAIT_RESUME",
    })

    result = step.execute(context)

    assert result["status"] == "error"
    assert result["move_status"] == "failed"
    assert context.get("move_status") == "failed"
