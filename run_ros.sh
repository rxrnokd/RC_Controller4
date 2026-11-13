#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname -- "$ROOT")"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$PROJECT/ros2_ws/install/setup.bash" ]]; then
  echo 'Existing sllidar workspace must be built first; see README.' >&2
  exit 1
fi
source "$PROJECT/ros2_ws/install/setup.bash"
set -u
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec ros2 launch "$ROOT/rc.launch.py"
