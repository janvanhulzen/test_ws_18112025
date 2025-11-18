from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_robot_world",
            executable="add_world_objects",
            name="add_world_objects",
            output="screen",
            # If you want to tune via params:
            # parameters=[{"planning_frame": "base_link"}],
        )
    ])
