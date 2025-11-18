# file: homing_bringup.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    homer = ExecuteProcess(
        cmd=[
            '/absolute/path/to/phidget_homer',
            '--stepper-serial','740266',
            '--stepper-ports','1,2,3,4,5',
            '--stepper-channels','0,0,0,0,0',
            '--daq-serial','528369',
            '--daq-port','0',
            '--di-min','0,1,2,3,4',
            '--di-max','11,12,13,14,15',
            '--accel','1500', '--vel','150', '--backoff','150'
        ],
        output='screen',
        shell=False
    )
    return LaunchDescription([homer])
