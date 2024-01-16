#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/foxy/setup.bash
source /opt/lane-capstone/install/setup.bash

ros2 launch ./launch/launch_detection.py &
/bin/bash
