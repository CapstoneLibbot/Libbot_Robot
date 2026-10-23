from __future__ import annotations

import json
import os
import time
from pathlib import Path

from core.tcp_server import TCPServer
from core.workflow_engine import WorkflowEngine
from runtime_gateway import RobotRuntimeGateway

ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "config" / "settings.json"


def load_settings() -> dict:
    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        return json.load(file)


def main() -> None:
    settings = load_settings()

    tcp_settings = settings.get("tcp", {})
    if os.getenv("LIBOT_TCP_PORT"):
        tcp_settings["port"] = int(os.environ["LIBOT_TCP_PORT"])
    server = TCPServer(tcp_settings)
    runtime_gateway = RobotRuntimeGateway(settings)
    workflow = WorkflowEngine(settings, server, runtime_gateway)

    def handle_request(payload: object) -> object:
        runtime_response = runtime_gateway.handle_request(payload)
        return runtime_response if runtime_response is not None else workflow.handle_request(payload)

    server.set_request_handler(handle_request)

    workflow.load_default_workflow()
    try:
        server.start_in_background()
        runtime_gateway.start()
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        runtime_gateway.stop()
        server.stop()


if __name__ == "__main__":
    main()
