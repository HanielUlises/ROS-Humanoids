"""
Top-level launcher for ROS-Humanoids.

Usage:
  ros2 launch launch/spawn_robot.launch.py robot:=g1
  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo
  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=rviz model:=29dof
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
    sim = LaunchConfiguration("sim").perform(context)

    repository_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    registry_path = os.path.join(repository_root, "config", "robots.yaml")
    with open(registry_path) as f:
        registry = yaml.safe_load(f)["robots"]

    if robot not in registry:
        raise ValueError(
            f"Unknown robot '{robot}'. Available: {sorted(registry)}"
        )

    entry = registry[robot]
    bringup = os.path.join(repository_root, entry["bringup"])
    robot_launch = os.path.join(bringup, f"{robot}_{sim}.launch.py")

    if not os.path.exists(robot_launch):
        available = sorted(
            name[len(robot) + 1:-len(".launch.py")]
            for name in os.listdir(bringup)
            if name.startswith(f"{robot}_") and name.endswith(".launch.py")
        )
        raise FileNotFoundError(
            f"No launch file for robot='{robot}' sim='{sim}' at:\n  {robot_launch}\n"
            f"Available backends for {robot}: {available}"
        )

    model = LaunchConfiguration("model").perform(context) or entry["default_model"]
    if model not in entry["urdf"]:
        raise ValueError(
            f"Unknown model '{model}' for '{robot}'. Available: {sorted(entry['urdf'])}"
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={"model": model}.items(),
        )
    ]


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
        DeclareLaunchArgument(
            "model",
            default_value="",
            description="Robot variant, e.g. 23dof | 29dof. Empty uses the registry default.",
        ),
        OpaqueFunction(function=_resolve_robot_launch),
    ])
