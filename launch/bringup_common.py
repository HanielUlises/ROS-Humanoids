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
