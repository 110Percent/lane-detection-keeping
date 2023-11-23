#!/bin/bash
set -e

# setup ros environment
source "/opt/lane-capstone/install/setup.bash"
source "/opt/carla-ros-bridge/install/setup.bash"
source "/opt/carla/setup.bash"

export NO_AT_BRIDGE=1

exec "$@"
