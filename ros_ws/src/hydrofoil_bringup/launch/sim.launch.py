"""Full simulator: Gazebo + hydrofoil craft + bridge + control node.
   ros2 launch hydrofoil_bringup sim.launch.py                 (with GUI)
   ros2 launch hydrofoil_bringup sim.launch.py headless:=true
   ros2 launch hydrofoil_bringup sim.launch.py physics:=v2     (phase 4 plant)
   ros2 launch hydrofoil_bringup sim.launch.py follow:=false   (free camera)
Then drive it with the `hf` command, e.g.  ros2 run hydrofoil_control hf speed 10
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("hydrofoil_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("scenario_json", default_value=""),
        DeclareLaunchArgument("duration", default_value="0.0"),
        DeclareLaunchArgument("log", default_value=""),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("physics", default_value="ref"),
        DeclareLaunchArgument("follow", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "foiling.launch.py")),
            launch_arguments={"headless": LaunchConfiguration("headless"),
                              "follow": LaunchConfiguration("follow"),
                              "physics": LaunchConfiguration("physics")}.items()),
        Node(package="hydrofoil_control", executable="env_node", output="screen",
             parameters=[{"seed": LaunchConfiguration("seed"),
                          "physics": ParameterValue(LaunchConfiguration("physics"), value_type=str)}]),
        Node(package="hydrofoil_control", executable="control_node", output="screen",
             parameters=[{"scenario_json": LaunchConfiguration("scenario_json"),
                          "duration": LaunchConfiguration("duration"),
                          "log": LaunchConfiguration("log")}]),
    ])
