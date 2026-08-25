"""Spawn the G1 in Gazebo Classic under the ros2_control stack.

  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo_control
  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo_control model:=29dof z:=1.0
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
from g1_common import robot_description  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "launch"))
from bringup_common import controller_parameters, with_ros2_control  # noqa: E402

CONTROLLERS = ["joint_pd_controller"]


def _nodes(context, *args, **kwargs):
    variant = LaunchConfiguration("model").perform(context)
    spawn_height = LaunchConfiguration("z").perform(context)

    controller_config = os.path.join(
        get_package_share_directory("g1_control"), "config", "g1_controllers.yaml"
    )
    # The controller manager runs inside the Gazebo plugin and reads its
    # parameters from the file the URDF points at, so the merged file is written
    # first and the description is composed around it.
    plain_description = robot_description(variant)
    parameters = controller_parameters(controller_config, plain_description, CONTROLLERS)
    description = with_ros2_control(
        plain_description, "gazebo_ros2_control/GazeboSystem", parameters
    )
    gazebo = os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[controller, "--controller-manager", "/controller_manager"],
        )
        for controller in ["joint_state_broadcaster"] + CONTROLLERS
    ]

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": description, "use_sim_time": True}],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=["-topic", "robot_description", "-entity", f"g1_{variant}", "-z", spawn_height],
        ),
        *spawners,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="23dof", description="23dof | 29dof"),
        DeclareLaunchArgument("z", default_value="0.8", description="spawn height [m]"),
        OpaqueFunction(function=_nodes),
    ])
