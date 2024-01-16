#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

ros2 launch carla_ros_bridge carla_ros_bridge.launch.py synchronous_mode:=false town:=town04 register_all_sensors:=false timeout:=10000 &
ros2 launch ./launch/launch_main.py &
/bin/bash
