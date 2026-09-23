from __future__ import annotations

from typing import Any, Dict, List

from core.context import RobotContext
from steps import (
    BaseStep,
    DisplayStep,
    EvaluateStep,
    MoveStep,
    NotifyStep,
    ProcessDataStep,
    ScanStep,
    WaitStep,
)


class WorkflowEngine:
    """Manages a list of steps and coordinates execution using RobotContext."""

    STEP_TYPES = {
        "MOVE": MoveStep,
        "SCAN": ScanStep,
        "PROCESS_DATA": ProcessDataStep,
        "EVALUATE": EvaluateStep,
        "DISPLAY": DisplayStep,
        "WAIT": WaitStep,
        "NOTIFY": NotifyStep,
    }

    def __init__(self, settings: Dict[str, Any], tcp_server: Any):
        self.settings = settings
        self.tcp_server = tcp_server
        self.context = RobotContext()
        self.steps: List[BaseStep] = []

    def load_default_workflow(self) -> None:
        workflow_steps = self.settings.get("workflow", {}).get("steps", [])
        self.steps = [self.create_step(step_def) for step_def in workflow_steps]

    def create_step(self, step_def: Dict[str, Any]) -> BaseStep:
        step_type = step_def.get("type", "WAIT")
        step_class = self.STEP_TYPES.get(step_type.upper(), WaitStep)
        return step_class(step_def)

    def run(self) -> None:
        for step in self.steps:
            result = step.execute(self.context)
            self.context.set_step_result(step.name, result)
            if self.tcp_server is not None:
                self.tcp_server.send(f"STEP:{step.name}:{result}")

    def add_step(self, step_def: Dict[str, Any]) -> None:
        self.steps.append(self.create_step(step_def))
