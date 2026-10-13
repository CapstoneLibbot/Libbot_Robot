from __future__ import annotations

from dataclasses import dataclass, field
import threading
from typing import Any, Dict


@dataclass
class RobotContext:
    """Stores runtime variables and results exchanged between workflow steps."""

    variables: Dict[str, Any] = field(default_factory=dict)
    step_results: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)
    events: list[Dict[str, Any]] = field(default_factory=list)
    lock: threading.RLock = field(default_factory=threading.RLock, repr=False)

    def set(self, key: str, value: Any) -> None:
        with self.lock:
            self.variables[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        with self.lock:
            return self.variables.get(key, default)

    def set_step_result(self, step_name: str, value: Any) -> None:
        with self.lock:
            self.step_results[step_name] = value

    def get_step_result(self, step_name: str, default: Any = None) -> Any:
        with self.lock:
            return self.step_results.get(step_name, default)

    def update_metadata(self, **kwargs: Any) -> None:
        with self.lock:
            self.metadata.update(kwargs)

    def update_variables(self, values: Dict[str, Any]) -> None:
        with self.lock:
            self.variables.update(values)

    def push_event(self, event_type: str, data: Any = None) -> None:
        with self.lock:
            self.events.append({"event_type": event_type, "data": data})

    def pop_event(self, event_type: str) -> Any:
        with self.lock:
            for index, event in enumerate(self.events):
                if event.get("event_type") == event_type:
                    return self.events.pop(index).get("data")
        return None

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "variables": dict(self.variables),
                "step_results": dict(self.step_results),
                "metadata": dict(self.metadata),
                "pending_events": list(self.events),
            }
