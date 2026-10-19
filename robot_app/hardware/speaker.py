from __future__ import annotations

import os
import shutil
import subprocess
from typing import Any, Dict, Optional


class SpeakerController:
    """Speech adapter for the robot speaker using the installed eSpeak executable."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}

    def say(self, message: str) -> None:
        if not self.settings.get("enabled", True):
            return
        executable = shutil.which("espeak-ng") or shutil.which("espeak")
        if not executable:
            raise RuntimeError("Install espeak-ng or espeak to enable robot speech.")
        volume = max(0, min(200, int(self.settings.get("volume", 70))))
        subprocess.run(
            [executable, "-a", str(volume), str(message)],
            check=True,
            timeout=float(self.settings.get("timeout_sec", 30)),
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
