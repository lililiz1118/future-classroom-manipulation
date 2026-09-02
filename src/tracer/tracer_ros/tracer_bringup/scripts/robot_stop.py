#!/usr/bin/env python3
import os
import sys


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.robot_stop import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
