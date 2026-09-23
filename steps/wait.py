from __future__ import annotations

import time
from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class WaitStep(BaseStep):
    """Step Type: WAIT"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.seconds = float(self.config.get("seconds", 0.0))

    def execute(self, context: RobotContext) -> Any:
        time.sleep(self.seconds)
        context.set("last_wait", self.seconds)
        return {
            "status": "waited",
            "type": "WAIT",
            "seconds": self.seconds,
        }
