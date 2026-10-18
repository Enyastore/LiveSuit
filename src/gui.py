"""LiveSuit GUI 入口。

保留 `python src/gui.py` 用法；实际实现位于 gui/ 包（gui.app:main）。
"""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from gui.app import main  # noqa: E402

if __name__ == "__main__":
    main()
