import time

from core.context import RobotContext
from core.workflow_engine import WorkflowEngine
from hardware.barcode_scanner import BarcodeScanner
from steps.end import EndStep
from steps.scan import ScanStep
from steps.process_data import ProcessDataStep


def test_end_step_respects_failed_status():
    result = EndStep({"type": "END", "status": "FAILED"}).execute(RobotContext())

    assert result["workflow_status"] == "FAILED"
    assert result["status"] == "failed"


def test_sim_scan_uses_fixture_queue_and_never_opens_physical_scanner(monkeypatch):
    context = RobotContext()
    context.set("runtime_id", "sim")
    context.set("scan_queue", ["BOOK-123"])
    monkeypatch.setattr(BarcodeScanner, "scan", lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("physical camera used in sim")))

    step = ScanStep({"type": "SCAN", "device": "imx500-tcp://127.0.0.1:9011", "timeout_sec": 0.01})

    assert step.execute(context)["value"] == "BOOK-123"
    assert step.execute(context)["status"] == "timeout"


def test_real_scan_does_not_consume_simulated_fixture(monkeypatch):
    context = RobotContext()
    context.set("runtime_id", "real")
    context.set("scan_queue", ["BOOK-123"])
    monkeypatch.setattr(BarcodeScanner, "scan", lambda *_args, **_kwargs: {"status": "timeout", "value": None})

    result = ScanStep({"type": "SCAN", "timeout_sec": 0.01}).execute(context)

    assert result["status"] == "timeout"
    assert context.get("scan_queue") == ["BOOK-123"]


def test_sim_search_uses_the_separate_fixture_endpoint(monkeypatch):
    requested_urls = []

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

        def read(self):
            return b'{"books": []}'

    def fake_urlopen(url, **_kwargs):
        requested_urls.append(getattr(url, "full_url", url))
        return Response()

    monkeypatch.setattr("steps.process_data.request.urlopen", fake_urlopen)
    step = ProcessDataStep({
        "type": "PROCESS_DATA",
        "action": "SEARCH_BOOK",
        "payload": {"query": "technology", "runtime_id": "sim"},
        "backend_base_url": "http://backend.local",
    })

    result = step.execute(RobotContext())

    assert result["status"] == "processed"
    assert "/api/workflow-data/simulation/books/search?" in requested_urls[0]


def test_workflow_validation_rejects_unknown_transition_targets():
    engine = WorkflowEngine({"runtime": {"id": "sim"}}, object())

    error = engine._validate_steps([
        {"id": "scan", "type": "SCAN", "transitions": {"on_success": "missing"}},
        {"id": "end", "type": "END"},
    ])

    assert error and "unknown step" in error


def test_execution_publishes_waiting_step_type_and_completes(monkeypatch):
    monkeypatch.delenv("LIBOT_BACKEND_URL", raising=False)
    engine = WorkflowEngine({"runtime": {"id": "sim"}}, object())
    published = []
    engine._publish_execution = lambda execution_id, status, result=None, error=None: published.append({
        "executionId": execution_id,
        "status": status,
        "result": result,
        "error": error,
        "stepType": engine.executions.get(execution_id, {}).get("current_step_type"),
        "stepId": engine.executions.get(execution_id, {}).get("current_step"),
    })

    response = engine.handle_request({
        "command": "start_workflow",
        "execution_id": "test-execution",
        "task_id": "test-mission",
        "runtime_id": "sim",
        "initial_context": {"runtime_id": "sim"},
        "steps": [
            {
                "id": "human-check",
                "name": "Human confirmation",
                "type": "WAIT",
                "wait_for": "USER_TOUCH",
                "event_type": "CONFIRM",
                "allowed_values": ["CONTINUE"],
                "result_variable": "answer",
                "timeout_sec": 2,
                "stop_motion": False,
            },
            {"id": "done", "type": "END", "status": "SUCCEEDED"},
        ],
    })
    assert response["status"] == "accepted"

    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        state = engine.executions.get("test-execution", {})
        if state.get("expected_event_type") == "CONFIRM":
            break
        time.sleep(0.01)
    assert engine.executions["test-execution"]["current_step_type"] == "WAIT"

    event_response = engine.handle_request({
        "command": "submit_event",
        "execution_id": "test-execution",
        "event_type": "CONFIRM",
        "data": {"value": "CONTINUE"},
    })
    assert event_response["status"] == "accepted"

    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and engine.executions["test-execution"].get("status") not in {"completed", "failed"}:
        time.sleep(0.01)

    assert engine.executions["test-execution"]["status"] == "completed"
    assert any(item["status"] == "WAITING" and item["stepType"] == "WAIT" for item in published)
    assert any(item["status"] == "RUNNING" and item["stepType"] == "END" for item in published)


def test_cancel_interrupts_wait_and_reports_cancelled(monkeypatch):
    monkeypatch.delenv("LIBOT_BACKEND_URL", raising=False)
    engine = WorkflowEngine({"runtime": {"id": "sim"}}, object())
    engine._publish_execution = lambda *_args, **_kwargs: None
    response = engine.handle_request({
        "command": "start_workflow",
        "execution_id": "cancel-execution",
        "task_id": "cancel-mission",
        "runtime_id": "sim",
        "initial_context": {"runtime_id": "sim"},
        "steps": [{
            "id": "wait",
            "type": "WAIT",
            "wait_for": "USER_TOUCH",
            "event_type": "CONFIRM",
            "timeout_sec": 30,
            "stop_motion": False,
        }],
    })
    assert response["status"] == "accepted"

    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and engine.executions["cancel-execution"].get("expected_event_type") != "CONFIRM":
        time.sleep(0.01)
    cancel_response = engine.handle_request({"command": "cancel_workflow", "execution_id": "cancel-execution"})
    assert cancel_response["status"] == "accepted"

    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and engine.executions["cancel-execution"].get("status") not in {"cancelled", "failed", "completed"}:
        time.sleep(0.01)

    assert engine.executions["cancel-execution"]["status"] == "cancelled"
