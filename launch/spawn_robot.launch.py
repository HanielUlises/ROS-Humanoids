"""
Top-level launcher for ROS-Humanoids.

Usage:
  ros2 launch launch/spawn_robot.launch.py robot:=g1
  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo
  ros2 launch launch/spawn_robot.launch.py robot:=h1 sim:=rviz
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_robot_launch(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    sim   = LaunchConfiguration("sim").perform(context)

    registry_path = os.path.join(
        os.path.dirname(__file__), "..", "config", "robots.yaml"
    )
    with open(registry_path) as f:
        registry = yaml.safe_load(f)

    if robot not in registry:
        raise ValueError(
            f"Unknown robot '{robot}'. Available: {list(registry.keys())}"
        )

    robot_launch = os.path.join(
        os.path.dirname(__file__),
        "..", "robots", robot, "bringup", f"{robot}_{sim}.launch.py",
    )

    if not os.path.exists(robot_launch):
        raise FileNotFoundError(
            f"No launch file for robot='{robot}' sim='{sim}' at:\n  {robot_launch}"
        )

    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_launch))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="g1",
            description="Robot to spawn. See config/robots.yaml for options.",
        ),
        DeclareLaunchArgument(
            "sim",
            default_value="rviz",
            description="Simulation backend: rviz | gazebo",
        ),
        OpaqueFunction(function=_resolve_robot_launch),
    ])
