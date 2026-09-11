#!/usr/bin/env bash
# Clone and build PX4 SITL for arm64 inside a container. ~25 min, once.
# The tree lands in px4/PX4-Autopilot (gitignored — it is 1.7 GB).
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
version="${PX4_VERSION:-v1.15.4}"

target="${PX4_DIR:-$here}"
mkdir -p "$target"
docker run --rm -v "$target":/px4 -w /px4 ubuntu:24.04 bash -lc "
  export DEBIAN_FRONTEND=noninteractive
  apt-get -qq update >/dev/null
  apt-get -qq install -y git cmake ninja-build build-essential python3 python3-pip \
      python3-jinja2 python3-yaml python3-numpy python3-setuptools python3-packaging \
      libeigen3-dev ca-certificates >/dev/null
  pip3 install -q --break-system-packages pyros-genmsg jsonschema empy==3.3.4 kconfiglib future
  [ -d PX4-Autopilot ] || git clone -q --depth 1 --recursive --branch $version \
      https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot && make px4_sitl_default
"
echo "built: $target/PX4-Autopilot/build/px4_sitl_default/bin/px4"
