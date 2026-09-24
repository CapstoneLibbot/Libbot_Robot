from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class NotifyStep(BaseStep):
    """Step Type: NOTIFY"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.message = self.config.get("message", "Notification")

    def execute(self, context: RobotContext) -> Any:
        context.set("last_notification", self.message)
        return {
            "status": "notified",
            "type": "NOTIFY",
            "message": self.message,
        }
