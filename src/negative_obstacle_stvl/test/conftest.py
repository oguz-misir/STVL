"""pytest sys.path ayarı — ROS dışı import için."""
import sys
from pathlib import Path

_src = Path(__file__).parent.parent.parent.parent
for p in [
    str(_src / "negative_obstacle_common"),
    str(_src / "negative_obstacle_stvl"),
]:
    if p not in sys.path:
        sys.path.insert(0, p)
