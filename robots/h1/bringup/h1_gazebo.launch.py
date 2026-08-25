"""Spawn the H1 in Gazebo Classic.

  ros2 launch launch/spawn_robot.launch.py robot:=h1 sim:=gazebo
  ros2 launch launch/spawn_robot.launch.py robot:=h1 sim:=gazebo z:=1.2
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from h1_common import robot_description  # noqa: E402


def _nodes(context, *args, **kwargs):
    variant = LaunchConfiguration("model").perform(context)
    spawn_height = LaunchConfiguration("z").perform(context)

    gazebo = os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description(variant), "use_sim_time": True}
            ],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-entity", f"h1_{variant}",
                "-z", spawn_height,
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="19dof", description="19dof"),
        DeclareLaunchArgument("z", default_value="1.1", description="spawn height [m]"),
        OpaqueFunction(function=_nodes),
    ])
