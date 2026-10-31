"""Generate the separately named open-frame quick-fit variant of the V7 mount."""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from check_fasteners import check
from export_mount import export
from generate_box import generate

if __name__ == "__main__":
    generate(HERE, lightweight=True)
    check(HERE)
    export(HERE, asset_name="mid360_raised_frame")
