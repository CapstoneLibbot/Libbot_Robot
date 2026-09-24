from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from core.context import RobotContext


class BaseStep(ABC):
    """Base class for all robot workflow steps."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = config or {}
        self.name = self.config.get("name", self.__class__.__name__)
        self.step_type = self.config.get("type", self.__class__.__name__.upper())

    @abstractmethod
    def execute(self, context: RobotContext) -> Any:
        """Execute step logic and return a result payload."""

    def read_value(self, context: RobotContext, key: str, default: Any = None) -> Any:
        return context.get(key, default)

    def write_value(self, context: RobotContext, key: str, value: Any) -> None:
        context.set(key, value)
