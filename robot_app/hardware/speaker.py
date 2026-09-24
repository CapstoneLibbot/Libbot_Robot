from __future__ import annotations

from typing import Any, Dict, Optional


class SpeakerController:
    """Interface for TTS and notification audio output."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}

    def say(self, message: str) -> None:
        print(f"Speaker: {message}")
