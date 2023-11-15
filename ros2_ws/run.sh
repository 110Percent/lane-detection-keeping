#!/bin/bash

ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py synchronous_mode:=false town:=town04 register_all_sensors:=false &
ros2 launch ./launch/test_launch.py &
/bin/bash