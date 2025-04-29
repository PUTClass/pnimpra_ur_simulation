import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description():
    ld = LaunchDescription()
    params = PathJoinSubstitution([FindPackageShare('ur_simulation'), 'config', 'simulation.yaml'])

    closest_objects_node = Node(
        package="ur_simulation",
        executable="closest_objects",
        name='closest_objects_node',
        parameters=[params]
    )
    ld.add_action(closest_objects_node)
    return ld