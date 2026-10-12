from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class RobotContext:
    """Stores runtime variables and results exchanged between workflow steps."""

    variables: Dict[str, Any] = field(default_factory=dict)
    step_results: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def set(self, key: str, value: Any) -> None:
        self.variables[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        return self.variables.get(key, default)

    def set_step_result(self, step_name: str, value: Any) -> None:
        self.step_results[step_name] = value

    def get_step_result(self, step_name: str, default: Any = None) -> Any:
        return self.step_results.get(step_name, default)

    def update_metadata(self, **kwargs: Any) -> None:
        self.metadata.update(kwargs)

    def update_variables(self, values: Dict[str, Any]) -> None:
        self.variables.update(values)

    def snapshot(self) -> Dict[str, Any]:
        return {
            "variables": dict(self.variables),
            "step_results": dict(self.step_results),
            "metadata": dict(self.metadata),
        }
