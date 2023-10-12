from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_nodes_py',
            namespace='test',
            executable='camera_controller',
            name='camera_controller'
        ),
        Node(
            package='lane_nodes_py',
            namespace='test',
            executable='detection',
            name='detection'
        ),
        Node(
            package='lane_nodes_py',
            namespace='test',
            executable='keeping',
            name='keeping'
        ),
        Node(
            package='lane_nodes_cpp',
            namespace='test',
            executable='movement_controller',
            name='movement_controller'
        )
    ])
