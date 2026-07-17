"""Run the TiPToP upper-computer client from a source checkout."""

from __future__ import annotations

import sys

from .cli import main


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
