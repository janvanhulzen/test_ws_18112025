from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Paths
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("my_robot_description"), "urdf", "skyentific_robot.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("my_robot_bringup"), "config", "robot_moveit.rviz"]
    )

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("my_robot_bringup"), "config", "skyentific_robot_controllers.yaml"]
    )

    qos_overrides_yaml = PathJoinSubstitution(
        [FindPackageShare("my_robot_bringup"), "config", "qos_overrides.yaml"]
    )

    safety_config_yaml = PathJoinSubstitution(
        [FindPackageShare("my_robot_safety"), "config", "limit_switch_watcher.yaml"]
    )

    moveit_rviz_params_yaml = PathJoinSubstitution(
        [FindPackageShare("my_robot_moveit_config"), "config", "rviz_params.yaml"]
    )

    move_group_launch = PathJoinSubstitution(
        [FindPackageShare("my_robot_moveit_config"), "launch", "move_group.launch.py"]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro", " ", urdf_path]),
            }
        ],
    )

    # ros2_control (controller_manager)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro", " ", urdf_path]),
            },
            controllers_yaml,
            qos_overrides_yaml,
        ],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_joints_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joints_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    hold_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hold_position_controller", "--inactive"],
        output="screen",
    )

    # Safety node
    safety_node = Node(
        package="my_robot_safety",
        executable="limit_switch_watcher_node",
        name="limit_switch_watcher",
        output="screen",
        parameters=[safety_config_yaml],
    )

    # MoveIt (include existing launch file)
    move_group_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch)
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config_path,
            "--ros-args",
            "--params-file",
            moveit_rviz_params_yaml,
        ],
    )

    # Add world objects
    add_world_objects_node = Node(
        package="my_robot_world",
        executable="add_world_objects",
        name="add_world_objects",
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            arm_joints_controller_spawner,
            gripper_controller_spawner,
            hold_position_controller_spawner,
            safety_node,
            move_group_include,
            rviz_node,
            add_world_objects_node,
        ]
    )
