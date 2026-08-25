"""Run the G1 control stack against mock hardware, without a simulator.

  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=mock
  ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=mock model:=29dof

Mock hardware reports back what is written to it, so this exercises the
controller manager, the plugin and the parameter wiring. It carries no
dynamics, and the joints stay where they start.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from g1_common import robot_description  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "launch"))
from bringup_common import controller_parameters, with_ros2_control  # noqa: E402

CONTROLLERS = ["joint_pd_controller"]


def _nodes(context, *args, **kwargs):
    variant = LaunchConfiguration("model").perform(context)

    description = with_ros2_control(
        robot_description(variant), "mock_components/GenericSystem"
    )
    controller_config = os.path.join(
        get_package_share_directory("g1_control"), "config", "g1_controllers.yaml"
    )
    parameters = controller_parameters(controller_config, description, CONTROLLERS)

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
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{"robot_description": description}, parameters],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": description}],
        ),
        *spawners,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="23dof", description="23dof | 29dof"),
        OpaqueFunction(function=_nodes),
    ])
