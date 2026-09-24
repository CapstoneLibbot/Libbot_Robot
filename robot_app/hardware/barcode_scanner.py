from __future__ import annotations

from typing import Any, Dict, Optional


class BarcodeScanner:
    """Interface for barcode or QR code scanning devices."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        self.settings = settings or {}

    def scan(self) -> Dict[str, Any]:
        return {"status": "idle", "value": None}
