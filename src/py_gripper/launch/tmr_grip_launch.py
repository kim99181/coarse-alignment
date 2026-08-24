from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        )
    )

    return LaunchDescription([
        Node(
            package='tm_driver',
            executable='tm_driver',
            name='tm_driver_node',
            arguments=['robot_ip:=192.168.10.31']
        ),

        realsense_launch,

        Node(
            package='py_gripper',
            executable='fp_pose_bridge',
            name='fp_pose_bridge'
        ),

        # Node(
        #     package='py_gripper',
        #     executable='arm_cmd',
        #     name='arm_cmd'
        # ),
    ])



