"""Shared helpers for the G1 bringup launch files."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

VARIANTS = ("23dof", "29dof")

# Source-tree fallback: robots/g1/bringup/ -> robots/g1/description/
_SOURCE_DESCRIPTION = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "description")
)


def description_share():
    """Directory holding the description, installed if available, source otherwise."""
    try:
        return get_package_share_directory("g1_description")
    except PackageNotFoundError:
        return _SOURCE_DESCRIPTION


def urdf_path(variant):
    if variant not in VARIANTS:
        raise ValueError(f"unknown G1 variant '{variant}', expected one of {VARIANTS}")
    path = os.path.join(description_share(), "urdf", f"g1_{variant}.urdf")
    if not os.path.exists(path):
        raise FileNotFoundError(f"missing URDF for the {variant} G1: {path}")
    return path


def robot_description(variant):
    with open(urdf_path(variant)) as urdf:
        return urdf.read()
