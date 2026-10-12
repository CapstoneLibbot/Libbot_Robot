from __future__ import annotations

import json
import time
from pathlib import Path

from core.tcp_server import TCPServer
from core.workflow_engine import WorkflowEngine

ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "config" / "settings.json"


def load_settings() -> dict:
    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        return json.load(file)


def main() -> None:
    settings = load_settings()

    tcp_settings = settings.get("tcp", {})
    server = TCPServer(tcp_settings)
    workflow = WorkflowEngine(settings, server)
    server.set_request_handler(workflow.handle_request)

    workflow.load_default_workflow()
    try:
        server.start_in_background()
        if workflow.steps:
            workflow.run()
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
