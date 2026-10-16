import sys
import time
import unittest
from pathlib import Path


ROBOT_APP = Path(__file__).resolve().parents[1] / "robot_app"
if str(ROBOT_APP) not in sys.path:
    sys.path.insert(0, str(ROBOT_APP))

from core.workflow_engine import WorkflowEngine


class FlowOneEngineTests(unittest.TestCase):
    def setUp(self):
        self.engine = WorkflowEngine(
            {"hardware": {"motion_safety": {"method": "disabled"}}},
            None,
        )

    def test_output_mapping_and_evaluate_transition(self):
        steps = [
            {
                "id": "S10",
                "name": "S10",
                "type": "PROCESS_DATA",
                "action": "IDENTITY",
                "payload": {"books": [{"bookId": "B1"}]},
                "output_mapping": {"data.books": "search_results"},
                "transitions": {"on_success": "S20"},
            },
            {
                "id": "S20",
                "name": "S20",
                "type": "EVALUATE",
                "left": "${choice}",
                "operator": "EQUAL",
                "right": "RATE",
                "transitions": {"on_true": "S40", "on_false": "S30"},
            },
            {"id": "S30", "name": "S30", "type": "DISPLAY", "ui_screen": "MAP_LOCATION"},
            {"id": "S40", "name": "S40", "type": "DISPLAY", "ui_screen": "RATING"},
        ]
        self.engine.context.set("choice", "RATE")

        result = self.engine.run_steps(steps)

        self.assertEqual("ok", result["status"])
        self.assertEqual([{"bookId": "B1"}], self.engine.context.get("search_results"))
        self.assertEqual(["S10", "S20", "S40"], [item["step"] for item in result["results"]])

    def test_wait_consumes_touch_event_and_writes_result_variable(self):
        self.engine.context.push_event("BOOK_SELECTED", {"selected_book_id": "book-123"})
        result = self.engine.execute_step({
            "name": "S30",
            "type": "WAIT",
            "wait_for": "USER_TOUCH",
            "event_type": "BOOK_SELECTED",
            "result_variable": "selected_book_id",
            "timeout_sec": 0.1,
            "stop_motion": False,
        })

        self.assertEqual("event", result["result"]["status"])
        self.assertEqual("book-123", self.engine.context.get("selected_book_id"))

    def test_start_workflow_is_asynchronous_and_queryable(self):
        response = self.engine.handle_request({
            "command": "start_workflow",
            "execution_id": "flow-1",
            "task_id": "task-1",
            "initial_context": {"query": "robotics"},
            "steps": [{"id": "S190", "name": "S190", "type": "DISPLAY", "ui_screen": "HOME"}],
        })
        self.assertEqual("accepted", response["status"])

        deadline = time.monotonic() + 1
        execution = None
        while time.monotonic() < deadline:
            execution = self.engine.handle_request({"command": "get_execution", "execution_id": "flow-1"})
            if execution["execution"]["status"] == "completed":
                break
            time.sleep(0.01)

        self.assertIsNotNone(execution)
        self.assertEqual("completed", execution["execution"]["status"])

    def test_book_selected_event_is_routed_to_execution_and_assigned_book(self):
        response = self.engine.handle_request({
            "command": "start_workflow",
            "execution_id": "book-flow-1",
            "task_id": "book-task-1",
            "initial_context": {"book_id": "book-123"},
            "steps": [{
                "id": "S30",
                "name": "S30",
                "type": "WAIT",
                "wait_for": "USER_TOUCH",
                "event_type": "BOOK_SELECTED",
                "result_variable": "selected_book_id",
                "timeout_sec": 3,
            }],
        })
        self.assertEqual("accepted", response["status"])

        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            execution = self.engine.handle_request({"command": "get_execution", "execution_id": "book-flow-1"})
            if execution["execution"].get("expectedEventType") == "BOOK_SELECTED":
                break
            time.sleep(0.01)

        wrong_execution = self.engine.handle_request({
            "command": "submit_event",
            "execution_id": "another-flow",
            "event_type": "BOOK_SELECTED",
            "data": {"selected_book_id": "book-123"},
        })
        self.assertEqual("error", wrong_execution["status"])

        wrong_book = self.engine.handle_request({
            "command": "submit_event",
            "execution_id": "book-flow-1",
            "event_type": "BOOK_SELECTED",
            "data": {"selected_book_id": "book-456"},
        })
        self.assertEqual("INVALID_EVENT_DATA", wrong_book["code"])

        selected = self.engine.handle_request({
            "command": "submit_event",
            "execution_id": "book-flow-1",
            "event_type": "BOOK_SELECTED",
            "data": {"selected_book_id": "book-123"},
        })
        self.assertEqual("accepted", selected["status"])

    def test_end_step_is_supported(self):
        response = self.engine.handle_request({
            "command": "run_step",
            "step": {"type": "END", "name": "END", "result_status": "SUCCEEDED", "output": {"task": "${task_id}"}},
        })
        self.assertEqual("SUCCEEDED", response["result"]["workflow_status"])

    def test_error_transition_can_show_home_but_execution_stays_failed(self):
        result = self.engine.run_steps([
            {
                "id": "S10",
                "name": "S10",
                "type": "PROCESS_DATA",
                "action": "UNKNOWN",
                "transitions": {"on_error": "S190"},
            },
            {"id": "S190", "name": "S190", "type": "DISPLAY", "ui_screen": "HOME"},
        ])

        self.assertEqual("error", result["status"])
        self.assertEqual(["S10", "S190"], [item["step"] for item in result["results"]])


if __name__ == "__main__":
    unittest.main()
