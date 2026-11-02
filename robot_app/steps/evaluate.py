from __future__ import annotations

import math
from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class EvaluateStep(BaseStep):
    """Step Type: EVALUATE"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.condition = self.config.get("condition", True)
        self.left = self.config.get("left")
        self.right = self.config.get("right")
        self.operator = str(self.config.get("operator", "EQUAL")).upper()

    def execute(self, context: RobotContext) -> Any:
        left = self.resolve(context, self.left)
        right = self.resolve(context, self.right)
        if self.left is None:
            evaluation = bool(self.resolve(context, self.condition))
        elif self.operator == "EQUAL":
            evaluation = left == right
        elif self.operator == "NOT_EQUAL":
            evaluation = left != right
        elif self.operator == "IN":
            evaluation = left in right if isinstance(right, (list, tuple, set, dict, str)) else False
        elif self.operator in {"LESS_THAN_OR_EQUAL", "GREATER_THAN"}:
            if isinstance(left, bool) or isinstance(right, bool):
                return {"status": "error", "type": "EVALUATE", "message": f"{self.operator} requires numeric values."}
            try:
                left_number = float(left)
                right_number = float(right)
            except (TypeError, ValueError, OverflowError):
                return {"status": "error", "type": "EVALUATE", "message": f"{self.operator} requires numeric values."}
            if not math.isfinite(left_number) or not math.isfinite(right_number):
                return {"status": "error", "type": "EVALUATE", "message": f"{self.operator} requires finite numeric values."}
            evaluation = left_number <= right_number if self.operator == "LESS_THAN_OR_EQUAL" else left_number > right_number
        else:
            return {"status": "error", "type": "EVALUATE", "message": f"Unsupported operator: {self.operator}"}
        context.set("last_evaluation", evaluation)
        return {
            "status": "evaluated",
            "type": "EVALUATE",
            "condition": self.condition,
            "left": left,
            "operator": self.operator,
            "right": right,
            "result": evaluation,
        }
