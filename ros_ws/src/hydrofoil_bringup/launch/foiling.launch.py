"""Gazebo + hydrofoil craft + gz<->ROS bridge (no controller).
   ros2 launch hydrofoil_bringup foiling.launch.py [headless:=true] [world:=foiling.sdf]
                                                   [follow:=true] [physics:=ref]
The craft is held on a rail at trim until the first foil command arrives.
With the GUI, the camera follows the craft (follow:=false to disable; the
offset is behind / to port / above, in metres).
"""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, AppendEnvironmentVariable, TimerAction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

FOLLOW_MSG = ('track_mode: FOLLOW, follow_target: {name: "hydrofoil_craft"}, '
              'follow_offset: {x: -7.5, y: 4, z: 2.4}, follow_pgain: 4.0')


def generate_launch_description():
    share = get_package_share_directory("hydrofoil_bringup")
    phys_share = get_package_share_directory("hydrofoil_physics")
    phys_lib = os.path.join(get_package_prefix("hydrofoil_physics"), "lib")
    world = [os.path.join(share, "worlds", ""), LaunchConfiguration("world")]
    headless = LaunchConfiguration("headless")
    follow_on = PythonExpression(["'", headless, "' != 'true' and '", LaunchConfiguration("follow"), "' == 'true'"])
    # the GUI needs a few seconds to come up; send the request a few times
    follow_cmd = ("for i in 1 2 3 4; do gz topic -t /gui/track -m gz.msgs.CameraTrack -p '"
                  + FOLLOW_MSG + "'; sleep 3; done")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("world", default_value="foiling.sdf"),
        DeclareLaunchArgument("follow", default_value="true"),
        DeclareLaunchArgument("physics", default_value=""),
        # read by the FoilDynamics plugin at start-up (empty = the SDF <physics> value)
        SetEnvironmentVariable("HYDROFOIL_PHYSICS", LaunchConfiguration("physics")),
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", phys_lib),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(phys_share, "models")),
        ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "3", world], output="screen",
                       condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v", "3", world], output="screen",
                       condition=IfCondition(headless)),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": os.path.join(share, "config", "bridge_hydrofoil.yaml")}],
             output="screen"),
        TimerAction(period=6.0, condition=IfCondition(follow_on),
                    actions=[ExecuteProcess(cmd=["bash", "-c", follow_cmd], output="log")]),
    ])
