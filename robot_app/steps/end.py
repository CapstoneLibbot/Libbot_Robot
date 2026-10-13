from __future__ import annotations

from typing import Any, Dict, Optional

from core.context import RobotContext
from steps.base_step import BaseStep


class EndStep(BaseStep):
    """Step Type: END - closes a workflow and exposes its final output."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.result_status = self.config.get("result_status", "SUCCEEDED")
        self.message = self.config.get("message")
        self.output = self.config.get("output", {})

    def execute(self, context: RobotContext) -> Dict[str, Any]:
        workflow_status = str(self.resolve(context, self.result_status)).upper()
        result = {
            "status": "ended" if workflow_status == "SUCCEEDED" else "failed",
            "type": "END",
            "workflow_status": workflow_status,
            "message": self.resolve(context, self.message),
            "final_output": self.resolve(context, self.output),
        }
        context.set("workflow_status", workflow_status)
        context.set("final_output", result["final_output"])
        return result
