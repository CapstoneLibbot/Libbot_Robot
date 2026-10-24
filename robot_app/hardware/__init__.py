"""Hardware abstraction layer for robot peripherals."""

from .barcode_scanner import BarcodeScanner
from .motion_safety import MotionSafetyController
from .navigation import NavigationController
from .speaker import SpeakerController
from .ui_controller import UIController

__all__ = [
    "NavigationController",
    "BarcodeScanner",
    "MotionSafetyController",
    "UIController",
    "SpeakerController",
]
