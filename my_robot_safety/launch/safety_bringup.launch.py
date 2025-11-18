# my_robot_safety/launch/safety_bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    safety_share = get_package_share_directory("my_robot_safety")

    di_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_share, "launch", "daq1301_inputs.launch.py")
        ),
        launch_arguments={
            "serial_number": "528369",            # <- set your DAQ1301_0 serial
            "channels": "[0,1,2,3,4,5,11,12,13,14,15]",
            "data_interval_ms": "4",
            "hub_port": "0",                    # uncomment if needed
            # "namespace": "safety",              # optional namespace
        }.items()
    )

    watcher = Node(
        package="my_robot_safety",
        executable="limit_switch_watcher_node",
        name="limit_switch_watcher",
        output="screen",
        parameters=[os.path.join(safety_share, "config", "limit_switch_watcher.yaml")],
    )

    return LaunchDescription([di_launch, watcher])
