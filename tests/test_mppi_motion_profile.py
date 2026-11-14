"""Checks the forward-oriented MPPI profile and its simulation overrides."""

import ast
from pathlib import Path

import yaml


BRINGUP = Path(__file__).resolve().parents[1] / "ros2_ws/src/libot_bringup"


def launch_defaults(name):
    tree = ast.parse((BRINGUP / "launch" / name).read_text(encoding="utf-8"))
    result = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        if not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                result[node.args[0].value] = keyword.value.value
    return result


def test_forward_oriented_omni_profile_keeps_obstacle_checks():
    params = yaml.safe_load((BRINGUP / "config/nav2.yaml").read_text(encoding="utf-8"))
    controller = params["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    assert follow["motion_model"] == "Omni"
    assert 0 < follow["vy_max"] < follow["vx_max"]
    assert 0 < follow["vy_std"] < follow["vx_std"]
    assert -follow["vx_max"] < follow["vx_min"] < 0
    assert follow["CostCritic"]["consider_footprint"]
    assert {"CostCritic", "PathAngleCritic", "PreferForwardCritic"} <= set(follow["critics"])
    assert follow["TwirlingCritic"]["cost_weight"] > 0
    assert "twirling_cost_weight" not in follow["TwirlingCritic"]
    assert controller["general_goal_checker"]["xy_goal_tolerance"] <= 0.1


def test_simulation_speed_and_horizon_fit_local_costmap():
    params = yaml.safe_load((BRINGUP / "config/nav2.yaml").read_text(encoding="utf-8"))
    follow = params["controller_server"]["ros__parameters"]["FollowPath"]
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    sim = launch_defaults("simulation_navigation.launch.py")
    vx, vy = float(sim["controller_vx_max"]), float(sim["controller_vy_max"])
    assert 0.20 < vx <= follow["vx_max"]
    assert 0 < vy < vx
    assert follow["time_steps"] * follow["model_dt"] * vx < local["width"] / 2


def test_obstacle_replanning_and_lateral_launch_overrides():
    params = yaml.safe_load((BRINGUP / "config/nav2.yaml").read_text(encoding="utf-8"))
    controller = params["controller_server"]["ros__parameters"]
    assert controller["progress_checker"]["movement_time_allowance"] == 8.0
    for name in ("navigation.launch.py", "simulation_navigation.launch.py"):
        defaults = launch_defaults(name)
        assert 0 < float(defaults["controller_vy_max"]) <= controller["FollowPath"]["vy_max"]
        assert 0 < float(defaults["controller_vy_std"]) <= controller["FollowPath"]["vy_std"]
    for name in ("local_costmap", "global_costmap"):
        costmap = params[name][name]["ros__parameters"]
        assert costmap["obstacle_layer"]["scan"]["max_obstacle_height"] >= 0.25
        assert costmap["obstacle_layer"]["scan"]["inf_is_valid"]
        assert costmap["obstacle_layer"]["scan"]["marking"]
        assert costmap["obstacle_layer"]["scan"]["clearing"]
        assert costmap["range_layer"]["enabled"]
