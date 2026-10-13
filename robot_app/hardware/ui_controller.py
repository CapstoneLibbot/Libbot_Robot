from __future__ import annotations

from typing import Any, Dict, Optional


class UIController:
    """Interface for visual display operations on the robot."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}

    def show(self, message: str) -> None:
        print(f"UI: {message}")

    def show_screen(self, screen: str, data: Any = None) -> None:
        print(f"UI[{screen}]: {data if data is not None else ''}")
