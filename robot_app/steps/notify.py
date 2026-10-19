from __future__ import annotations

import subprocess
from typing import Any, Dict, Optional

from core.context import RobotContext
from hardware.speaker import SpeakerController
from steps.base_step import BaseStep


class NotifyStep(BaseStep):
    """Step Type: NOTIFY"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.message = self.config.get("content", self.config.get("message", "Notification"))
        self.channel = self.config.get("channel", "LOG")
        self.speaker = SpeakerController(self.config.get("speaker", {}))

    def execute(self, context: RobotContext) -> Any:
        message = self.resolve(context, self.message)
        if str(self.channel).upper() == "SPEAKER_TTS":
            try:
                self.speaker.say(str(message))
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                return {
                    "status": "error",
                    "type": "NOTIFY",
                    "channel": self.channel,
                    "message": str(message),
                    "error": str(exc),
                }
        context.set("last_notification", message)
        return {
            "status": "notified",
            "type": "NOTIFY",
            "channel": self.channel,
            "message": message,
        }
