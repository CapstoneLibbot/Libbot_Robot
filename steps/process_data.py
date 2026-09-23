from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class ProcessDataStep(BaseStep):
    """Step Type: PROCESS_DATA"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.operation = self.config.get("operation", "identity")

    def execute(self, context: RobotContext) -> Any:
        source = context.get("last_scan")
        result = {
            "status": "processed",
            "type": "PROCESS_DATA",
            "operation": self.operation,
            "source": source,
        }
        context.set("processed_data", result)
        return result
