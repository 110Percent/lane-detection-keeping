#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

ros2 launch carla_ros_bridge carla_ros_bridge.launch.py synchronous_mode:=false town:=town04 register_all_sensors:=false &
ros2 launch ./launch/test_launch.py objects_definition_file:='./../carla_config/objects.json' &
/bin/bash
