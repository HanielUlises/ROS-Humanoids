"""Description lookup for the H1 bringup launch files."""

import os
import sys

_REPOSITORY_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")
)
sys.path.insert(0, os.path.join(_REPOSITORY_ROOT, "launch"))

import bringup_common  # noqa: E402

PACKAGE = "h1_description"
SOURCE_DIRECTORY = "robots/h1/description"
VARIANTS = {"19dof": "h1.urdf"}


def description_directory():
    return bringup_common.description_share(PACKAGE, SOURCE_DIRECTORY)


def robot_description(variant):
    if variant not in VARIANTS:
        raise ValueError(f"unknown H1 variant '{variant}', expected one of {sorted(VARIANTS)}")
    return bringup_common.robot_description(PACKAGE, SOURCE_DIRECTORY, VARIANTS[variant])
