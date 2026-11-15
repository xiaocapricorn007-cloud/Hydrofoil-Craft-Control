"""Start Gazebo with the ocean world and the gz<->ROS bridge.
   ros2 launch hydrofoil_bringup ocean.launch.py            (GUI)
   ros2 launch hydrofoil_bringup ocean.launch.py headless:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hydrofoil_bringup")
    world = os.path.join(share, "worlds", "ocean.sdf")
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    headless = LaunchConfiguration("headless")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": f"-r {world}"}.items(),
                                 condition=UnlessCondition(headless)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": f"-r -s {world}"}.items(),
                                 condition=IfCondition(headless)),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml"),
                          "use_sim_time": True}],
             output="screen"),
    ])
