from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class MoveStep(BaseStep):
    """Step Type: MOVE"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.target = self.config.get("target", {"x": 0, "y": 0})
        self.speed = self.config.get("speed", 1.0)

    def execute(self, context: RobotContext) -> Any:
        destination = self.target or {"x": 0, "y": 0}
        context.set("last_move_target", destination)
        return {
            "status": "ok",
            "type": "MOVE",
            "target": destination,
            "speed": self.speed,
        }
