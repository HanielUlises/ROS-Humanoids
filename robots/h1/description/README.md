# Unitree H1 Description (URDF & MJCF)

Robot description for the [Unitree H1](https://www.unitree.com/h1), a 19-DOF
humanoid.

## Contents

- `urdf/h1.urdf`, the 19-DOF H1 without hands
- `mjcf/h1.xml` and `mjcf/scene.xml`, the MuJoCo model and its scene
- `meshes/`, the 21 STL meshes referenced by both

## Provenance

Taken from [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)
(`robots/h1_description`), BSD 3-Clause, see `LICENSE`.

Three changes were made to the upstream package:

- the dexterous hand variant (`h1_with_hand`) and its meshes are omitted
- the Collada meshes are omitted, since neither the URDF nor the MJCF uses them
- `package.xml` and `CMakeLists.txt` are the dual ROS 1 / ROS 2 versions used by
  the other descriptions in this repository

## Structure

19 revolute joints: five per leg (hip pitch, roll and yaw, knee, ankle), four
per arm (shoulder pitch, roll and yaw, elbow) and a torso yaw joint.
