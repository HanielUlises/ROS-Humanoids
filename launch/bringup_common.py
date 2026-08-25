"""Helpers shared by the per-robot bringup launch files."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

REPOSITORY_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def description_share(package, source_directory):
    """Directory holding a description, installed if available, source otherwise."""
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        return os.path.join(REPOSITORY_ROOT, source_directory)


def urdf_path(package, source_directory, urdf_name):
    path = os.path.join(description_share(package, source_directory), "urdf", urdf_name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"missing URDF: {path}")
    return path


def robot_description(package, source_directory, urdf_name):
    with open(urdf_path(package, source_directory, urdf_name)) as urdf:
        return urdf.read()


def actuated_joints(urdf_xml):
    """Names of the revolute, continuous and prismatic joints of a URDF."""
    from xml.etree import ElementTree

    root = ElementTree.fromstring(urdf_xml)
    return [
        joint.get("name")
        for joint in root.iter("joint")
        if joint.get("type") in ("revolute", "continuous", "prismatic")
    ]


def with_ros2_control(urdf_xml, hardware_plugin, controller_config=None, initial_positions=None):
    """Return the URDF with a ros2_control section for every actuated joint.

    The descriptions are vendored upstream material and carry no ros2_control
    tags, so the section is composed here and the files stay untouched. Each
    joint gets an effort command interface and position, velocity and effort
    state interfaces.
    """
    from xml.etree import ElementTree

    root = ElementTree.fromstring(urdf_xml)
    initial_positions = initial_positions or {}

    control = ElementTree.SubElement(
        root, "ros2_control", {"name": f"{root.get('name')}_system", "type": "system"}
    )
    hardware = ElementTree.SubElement(control, "hardware")
    ElementTree.SubElement(hardware, "plugin").text = hardware_plugin

    for joint in actuated_joints(urdf_xml):
        entry = ElementTree.SubElement(control, "joint", {"name": joint})
        ElementTree.SubElement(entry, "command_interface", {"name": "effort"})
        position = ElementTree.SubElement(entry, "state_interface", {"name": "position"})
        ElementTree.SubElement(position, "param", {"name": "initial_value"}).text = str(
            initial_positions.get(joint, 0.0)
        )
        ElementTree.SubElement(entry, "state_interface", {"name": "velocity"})
        ElementTree.SubElement(entry, "state_interface", {"name": "effort"})

    if controller_config is not None:
        gazebo = ElementTree.SubElement(root, "gazebo")
        plugin = ElementTree.SubElement(
            gazebo,
            "plugin",
            {"filename": "libgazebo_ros2_control.so", "name": "gazebo_ros2_control"},
        )
        ElementTree.SubElement(plugin, "parameters").text = controller_config

    return ElementTree.tostring(root, "unicode")


def controller_parameters(controller_config, robot_description, controllers):
    """Write a parameter file that adds the URDF to the given controllers.

    The controllers load the model themselves, and a spawner can only pass
    parameters as a file, so the two sources are merged into one temporary file.
    """
    import tempfile

    import yaml

    with open(controller_config) as configured:
        parameters = yaml.safe_load(configured) or {}

    for controller in controllers:
        entry = parameters.setdefault(controller, {}).setdefault("ros__parameters", {})
        entry["robot_description"] = robot_description

    handle, path = tempfile.mkstemp(prefix="controllers_", suffix=".yaml")
    with os.fdopen(handle, "w") as merged:
        yaml.safe_dump(parameters, merged)
    return path
