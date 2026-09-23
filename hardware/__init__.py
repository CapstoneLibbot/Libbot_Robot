"""Hardware abstraction layer for robot peripherals."""

from .barcode_scanner import BarcodeScanner
from .navigation import NavigationController
from .speaker import SpeakerController
from .ui_controller import UIController

__all__ = ["NavigationController", "BarcodeScanner", "UIController", "SpeakerController"]
