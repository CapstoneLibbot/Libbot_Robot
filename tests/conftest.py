import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT_APP = ROOT / "robot_app"

if str(ROBOT_APP) not in sys.path:
    sys.path.insert(0, str(ROBOT_APP))
