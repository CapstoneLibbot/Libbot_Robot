#!/usr/bin/env python3
"""Send one map-frame NavigateToPose goal through Nav2 Simple Commander."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional


def positive_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return number


def make_pose(navigator, x: float, y: float, yaw: float):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def result_name(result, task_result) -> str:
    if result == task_result.SUCCEEDED:
        return "SUCCEEDED"
    if result == task_result.CANCELED:
        return "CANCELED"
    if result == task_result.FAILED:
        return "FAILED"
    return f"UNKNOWN ({result!r})"


def wait_for_terminal_result(navigator, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if navigator.isTaskComplete():
            return True
        time.sleep(0.1)
    return navigator.isTaskComplete()


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Initialize AMCL (unless skipped), wait for Nav2, and navigate to one "
            "goal in the currently loaded map. X/Y are meters; yaw values are radians."
        )
    )
    parser.add_argument("--initial-x", type=float, help="verified robot X in map frame (m)")
    parser.add_argument("--initial-y", type=float, help="verified robot Y in map frame (m)")
    parser.add_argument("--initial-yaw", type=float, help="verified robot heading (rad)")
    parser.add_argument(
        "--skip-initial-pose",
        action="store_true",
        help="use only when localization is already established (for example, SIM ground truth)",
    )
    parser.add_argument("--goal-x", type=float, required=True, help="goal X in map frame (m)")
    parser.add_argument("--goal-y", type=float, required=True, help="goal Y in map frame (m)")
    parser.add_argument("--goal-yaw", type=float, required=True, help="goal heading (rad)")
    parser.add_argument(
        "--timeout",
        type=positive_float,
        default=90.0,
        help="maximum active-goal time in seconds (default: 90)",
    )
    parser.add_argument(
        "--feedback-interval",
        type=positive_float,
        default=1.0,
        help="minimum seconds between printed feedback updates (default: 1)",
    )
    parser.add_argument(
        "--cancel-result-timeout",
        type=positive_float,
        default=15.0,
        help="seconds to wait for a terminal result after cancellation (default: 15)",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)

    initial_values = (args.initial_x, args.initial_y, args.initial_yaw)
    if args.skip_initial_pose and any(value is not None for value in initial_values):
        parser.error("do not pass initial pose values together with --skip-initial-pose")
    if not args.skip_initial_pose and any(value is None for value in initial_values):
        parser.error("--initial-x, --initial-y and --initial-yaw are required unless --skip-initial-pose is used")

    numeric_values = [args.goal_x, args.goal_y, args.goal_yaw]
    if not args.skip_initial_pose:
        numeric_values.extend(initial_values)
    if not all(math.isfinite(value) for value in numeric_values):
        parser.error("pose values must be finite numbers")

    try:
        import rclpy
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    except ImportError as exc:
        print(f"ROS/Nav2 Python packages are unavailable: {exc}", file=sys.stderr)
        return 2

    navigator = None
    task_started = False
    timed_out = False
    try:
        rclpy.init(args=None)
        navigator = BasicNavigator()

        if not args.skip_initial_pose:
            initial_pose = make_pose(navigator, *initial_values)
            navigator.setInitialPose(initial_pose)
            print("Initial pose sent in frame 'map'.", flush=True)
        else:
            print("Using the existing localization/ground-truth pose.", flush=True)

        # This blocks until the localization and navigation lifecycle nodes are active.
        navigator.waitUntilNav2Active()

        goal_pose = make_pose(navigator, args.goal_x, args.goal_y, args.goal_yaw)
        navigator.goToPose(goal_pose)
        task_started = True
        # Start the application timeout only after Simple Commander submits the goal.
        started = time.monotonic()
        next_feedback_at = started
        print("Navigation goal submitted; waiting for feedback/result.", flush=True)

        while not navigator.isTaskComplete():
            now = time.monotonic()
            if now - started >= args.timeout:
                timed_out = True
                print("Goal timeout reached; requesting cancellation.", file=sys.stderr, flush=True)
                navigator.cancelTask()
                if not wait_for_terminal_result(navigator, args.cancel_result_timeout):
                    print(
                        "Cancellation was requested, but Nav2 did not return a terminal result "
                        f"within {args.cancel_result_timeout:g} seconds.",
                        file=sys.stderr,
                    )
                    return 124
                break

            feedback = navigator.getFeedback()
            if feedback is not None and now >= next_feedback_at:
                distance = getattr(feedback, "distance_remaining", None)
                try:
                    distance = float(distance)
                    if not math.isfinite(distance):
                        distance = None
                except (TypeError, ValueError, OverflowError):
                    distance = None
                if distance is not None:
                    print(f"Distance remaining: {distance:.2f} m", flush=True)
                    next_feedback_at = now + args.feedback_interval
            time.sleep(0.1)

        result = navigator.getResult()
        name = result_name(result, TaskResult)
        print(f"Nav2 terminal result: {name}", flush=True)
        if timed_out:
            return 124
        return 0 if result == TaskResult.SUCCEEDED else 1

    except KeyboardInterrupt:
        print("Interrupted; requesting cancellation.", file=sys.stderr, flush=True)
        if navigator is not None and task_started:
            navigator.cancelTask()
            if wait_for_terminal_result(navigator, args.cancel_result_timeout):
                print(
                    f"Nav2 terminal result: {result_name(navigator.getResult(), TaskResult)}",
                    flush=True,
                )
            else:
                print("No terminal result received after cancellation.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Navigation client failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if navigator is not None:
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
