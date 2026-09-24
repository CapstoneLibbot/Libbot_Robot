from __future__ import annotations

from typing import Any, Dict, Optional


class NavigationController:
    """Interface for movement and route control."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}

    def move_to(self, x: float, y: float) -> Dict[str, float]:
        return {"x": x, "y": y}

    def stop(self) -> None:
        pass
