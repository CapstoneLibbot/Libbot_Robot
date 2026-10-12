import threading
import time

from core.context import RobotContext
from core.tcp_server import TCPServer
from core.workflow_engine import WorkflowEngine
from steps.scan import ScanStep
from steps.wait import WaitStep


def test_scan_step_reads_runtime_context_value():
    context = RobotContext()
    context.set("scan_input", "BOOK-123")

    step = ScanStep({
        "expected_prefix": "BOOK-",
        "output_key": "book_barcode",
    })

    result = step.execute(context)

    assert result["status"] == "scanned"
    assert result["type"] == "SCAN"
    assert result["value"] == "BOOK-123"
    assert context.get("book_barcode") == "BOOK-123"
    assert context.get("last_scan")["source"] == "scan_input"


def test_scan_step_uses_mock_scanner_value():
    context = RobotContext()
    step = ScanStep({
        "mock_value": "RFID-001",
        "device": "mock",
    })

    result = step.execute(context)

    assert result["status"] == "scanned"
    assert result["value"] == "RFID-001"
    assert result["source"] == "mock"


def test_wait_step_waits_for_context_without_motion_stop():
    context = RobotContext()
    step = WaitStep({
        "context_key": "touch_confirmed",
        "expected_value": True,
        "timeout_sec": 1.0,
        "poll_interval_sec": 0.01,
        "stop_motion": False,
    })

    def confirm_later():
        time.sleep(0.05)
        context.set("touch_confirmed", True)

    thread = threading.Thread(target=confirm_later)
    thread.start()
    result = step.execute(context)
    thread.join()

    assert result["status"] == "waited"
    assert result["context_key"] == "touch_confirmed"
    assert context.get("last_wait_result")["status"] == "waited"


def test_tcp_server_processes_json_with_workflow_handler():
    settings = {
        "hardware": {
            "scanner": {"device": "camera", "timeout_sec": 0},
            "motion_safety": {"method": "disabled"},
        },
        "workflow": {"steps": []},
    }
    engine = WorkflowEngine(settings, None)
    server = TCPServer(request_handler=engine.handle_request)

    response = server.process_message(
        '{"command":"run_step","step":{"type":"SCAN","mock_value":"BOOK-999"}}'
    )

    assert response["status"] == "ok"
    assert response["result"]["type"] == "SCAN"
    assert response["result"]["value"] == "BOOK-999"


def test_workflow_handler_can_set_context_and_run_wait():
    settings = {
        "hardware": {
            "motion_safety": {"method": "disabled"},
        },
        "workflow": {"steps": []},
    }
    engine = WorkflowEngine(settings, None)

    set_response = engine.handle_request({
        "command": "set_context",
        "values": {"ready": True},
    })
    run_response = engine.handle_request({
        "command": "run_step",
        "step": {
            "type": "WAIT",
            "context_key": "ready",
            "expected_value": True,
            "timeout_sec": 0.1,
        },
    })

    assert set_response["status"] == "ok"
    assert run_response["status"] == "ok"
    assert run_response["result"]["status"] == "waited"
