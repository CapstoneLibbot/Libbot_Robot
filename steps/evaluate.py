from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class EvaluateStep(BaseStep):
    """Step Type: EVALUATE"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.condition = self.config.get("condition", True)

    def execute(self, context: RobotContext) -> Any:
        evaluation = bool(self.condition)
        context.set("last_evaluation", evaluation)
        return {
            "status": "evaluated",
            "type": "EVALUATE",
            "condition": self.condition,
            "result": evaluation,
        }
