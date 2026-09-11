"""Locate the plain-numpy flight package and put it on sys.path.

The flight code (`drone/`) deliberately lives outside the ROS package so the
same modules back both the standalone CLI and these nodes. colcon copies the
node sources into install/.../site-packages/, so a fixed number of `.parent`
hops resolves differently before and after a build. Walk up looking for the
package instead, and let an env var override for unusual layouts.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def add_flight_package_to_path() -> Path:
    candidates: list[Path] = []

    override = os.environ.get("DRONE_REPO")
    if override:
        candidates.append(Path(override))

    # Walk up from this file, and from the working directory, looking for the
    # marker file. Covers running from source, from an install tree, and from
    # a mounted volume at an arbitrary depth.
    for start in (Path(__file__).resolve(), Path.cwd().resolve() / "_"):
        candidates.extend(start.parents)

    for base in candidates:
        if (base / "drone" / "dynamics.py").is_file():
            if str(base) not in sys.path:
                sys.path.insert(0, str(base))
            return base

    raise ImportError(
        "could not find the flight package (drone/dynamics.py). "
        "Set DRONE_REPO to the repository root."
    )
