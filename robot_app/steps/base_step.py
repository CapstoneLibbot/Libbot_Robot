from __future__ import annotations

from abc import ABC, abstractmethod
import re
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

    def resolve(self, context: RobotContext, value: Any) -> Any:
        if isinstance(value, dict):
            return {key: self.resolve(context, item) for key, item in value.items()}
        if isinstance(value, list):
            return [self.resolve(context, item) for item in value]
        if not isinstance(value, str):
            return value

        full_match = re.fullmatch(r"\$\{([^}]+)\}", value)
        if full_match:
            return self._read_path(context, full_match.group(1))

        def replace(match: re.Match[str]) -> str:
            resolved = self._read_path(context, match.group(1))
            return "" if resolved is None else str(resolved)

        return re.sub(r"\$\{([^}]+)\}", replace, value)

    def _read_path(self, context: RobotContext, path: str) -> Any:
        parts = path.split(".")
        value = context.get(parts[0])
        for part in parts[1:]:
            if isinstance(value, dict):
                value = value.get(part)
            else:
                return None
        return value
