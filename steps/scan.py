from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class ScanStep(BaseStep):
    """Step Type: SCAN"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.sensor = self.config.get("sensor", "camera")

    def execute(self, context: RobotContext) -> Any:
        value = {
            "status": "scanned",
            "type": "SCAN",
            "sensor": self.sensor,
            "result": "no_data",
        }
        context.set("last_scan", value)
        return value
