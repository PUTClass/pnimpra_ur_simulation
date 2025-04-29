import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    params = PathJoinSubstitution([FindPackageShare('ur_simulation'), 'config', 'simulation.yaml'])

    detect_shapes_node = Node(
        package="ur_simulation",
        executable="detect_shapes",
        name='detect_shapes_node',
        parameters=[params]

    )
    ld.add_action(detect_shapes_node)
    return ld