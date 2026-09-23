from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class DisplayStep(BaseStep):
    """Step Type: DISPLAY"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.message = self.config.get("message", "Robot ready")

    def execute(self, context: RobotContext) -> Any:
        context.set("display_message", self.message)
        return {
            "status": "displayed",
            "type": "DISPLAY",
            "message": self.message,
        }
