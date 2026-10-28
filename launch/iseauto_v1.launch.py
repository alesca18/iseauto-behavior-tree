from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    episodes = LaunchConfiguration("episodes")
    return LaunchDescription([
        DeclareLaunchArgument("episodes", default_value="50"),
        Node(package="iseauto_bt", executable="metadrive_bridge.py", output="screen",
             arguments=["--episodes", episodes]),
        Node(package="iseauto_bt", executable="brain_node", output="screen"),
    ])
