#!/usr/bin/env bash
# Fly the real PX4 firmware: arm, climb, translate, return, land.
#
# Uses px4/PX4-Autopilot by default; set PX4_DIR to point at a tree built
# elsewhere (it is 1.7 GB, so keeping it outside the repo is reasonable).
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
px4_dir="${PX4_DIR:-$here/PX4-Autopilot}"

[ -x "$px4_dir/build/px4_sitl_default/bin/px4" ] || {
    echo "PX4 not built at $px4_dir — run ./px4/build.sh first" >&2
    exit 1
}

docker run --rm \
    -v "$px4_dir":/px4 \
    -v "$here":/scripts \
    -w /px4 \
    ubuntu:24.04 bash /scripts/_fly_inner.sh
