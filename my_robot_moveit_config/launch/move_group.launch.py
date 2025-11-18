from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


""" def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("skyentific_robot", package_name="my_robot_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config) """

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("skyentific_robot", package_name="my_robot_moveit_config")
        .robot_description()                # ensure URDF is loaded
        .robot_description_semantic()       # SRDF
        .robot_description_kinematics()     # kinematics.yaml
        .joint_limits("config/joint_limits.yaml")  # <-- your YAML becomes robot_description_planning
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)




""" # my_robot_moveit_config/launch/move_group.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    pkg = get_package_share_directory("my_robot_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("skyentific_robot", package_name="my_robot_moveit_config")
        .to_moveit_configs()
    )

    controllers_yaml = os.path.join(pkg, "config", "controllers.yaml")
    # (optional) QoS overrides if you made one
    qos_yaml = os.path.join(pkg, "config", "qos_overrides.yaml")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),              # all standard MoveIt params
            controllers_yaml,                     # <-- ensure controllers are loaded
            # qos_yaml,                           # <-- uncomment if you have it
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
        ],
    )

    return LaunchDescription([move_group_node])
 """