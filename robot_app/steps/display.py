from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from hardware.ui_controller import UIController
from steps.base_step import BaseStep


class DisplayStep(BaseStep):
    """Step Type: DISPLAY"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.message = self.config.get("message", "Robot ready")
        self.ui_screen = self.config.get("ui_screen", "MESSAGE")
        self.data = self.config.get("data", {})
        self.ui = UIController(self.config.get("ui", {}))

    def execute(self, context: RobotContext) -> Any:
        data = self.resolve(context, self.data)
        message = self.resolve(context, self.message)
        self.ui.show_screen(self.ui_screen, data)
        context.set("display_message", message)
        context.set("current_screen", self.ui_screen)
        return {
            "status": "displayed",
            "type": "DISPLAY",
            "ui_screen": self.ui_screen,
            "message": message,
            "data": data,
        }
