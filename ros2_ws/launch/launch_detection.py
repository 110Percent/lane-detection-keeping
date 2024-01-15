from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    detection_node = Node(
            package='lane_nodes_py',
            namespace='test',
            executable='detection_node',
            name='detection'
            )

    return LaunchDescription([
        detection_node
        ])
