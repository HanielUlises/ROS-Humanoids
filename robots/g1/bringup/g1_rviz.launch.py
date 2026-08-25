"""Show the G1 in RViz, with sliders for every joint.

  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=rviz
  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=rviz model:=29dof
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from g1_common import description_share, robot_description  # noqa: E402


def _nodes(context, *args, **kwargs):
    variant = LaunchConfiguration("model").perform(context)
    use_gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1")

    rviz_config = os.path.join(description_share(), "launch", "check_joint.rviz")
    joint_state_publisher = "joint_state_publisher_gui" if use_gui else "joint_state_publisher"

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description(variant)}],
        ),
        Node(
            package=joint_state_publisher,
            executable=joint_state_publisher,
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="23dof", description="23dof | 29dof"),
        DeclareLaunchArgument("gui", default_value="true", description="joint sliders"),
        OpaqueFunction(function=_nodes),
    ])
