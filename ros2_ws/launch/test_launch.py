import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        Node(
            package='lane_nodes_py',
            namespace='test',
            executable='camera_controller_node',
            name='camera_controller'
        ),
        Node(
            package='lane_nodes_py',
            namespace='test',
            executable='detection_node',
            name='detection'
        ),
        Node(
            package='lane_nodes_py',
            namespace='test',
            executable='keeping_node',
            name='keeping'
        ),
        Node(
            package='lane_nodes_cpp',
            namespace='test',
            executable='movement_controller',
            name='movement_controller'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ackermann_control'), 'carla_ackermann_control.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
