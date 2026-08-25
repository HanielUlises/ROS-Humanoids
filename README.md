# ROS-Humanoids

A small, opinionated ROS 2 workspace for humanoid robots. One registry, one
launcher, one command to put a robot on screen. Right now the resident is the
[Unitree G1](https://www.unitree.com/g1), a 35 kg biped with 23 or 29 joints
depending on how much wrist and waist you want.

![The Unitree G1 standing in MuJoCo](robots/g1/g1_model/docs/g1_mujoco.png)

The idea is boring on purpose: adding a robot should mean dropping a description
package next to the others, writing a bringup launch file, and adding a few
lines to `config/robots.yaml`. Everything else, including the top level
launcher, keeps working without being told about it.

## Put a robot on screen

```bash
colcon build
source install/setup.bash

ros2 launch launch/spawn_robot.launch.py robot:=g1                      # RViz, 23 DOF
ros2 launch launch/spawn_robot.launch.py robot:=g1 model:=29dof         # RViz, 29 DOF
ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo z:=1.0   # Gazebo Classic
```

`robot:` and `sim:` are looked up in the registry, so a typo tells you what is
actually available instead of failing somewhere deep in a launch include.

## What lives where

| Path | What it holds |
| --- | --- |
| `config/robots.yaml` | the registry: every robot, its variants, its description and its bringup directory |
| `launch/spawn_robot.launch.py` | the one entry point, which resolves the registry and hands off to a robot's bringup |
| `robots/g1/description/` | URDF, MJCF, meshes and RViz config for the G1 |
| `robots/g1/bringup/` | the RViz and Gazebo launch files the top level dispatches to |
| `robots/g1/g1_model/` | a C++ library that turns those URDFs into joint limits, forward kinematics and a centre of mass |

## The robots

| Robot | Variants | Mass | Description | Sim | Real hardware |
| --- | --- | --- | --- | --- | --- |
| Unitree G1 | 23 DOF, 29 DOF | 34.1 kg, 35.1 kg | URDF and MJCF | RViz, Gazebo | not wired up yet |

The 23-DOF build has a fixed waist and roll-only wrists. The 29-DOF build adds
waist roll and pitch plus two extra joints per wrist, which is what the picture
above is showing off.

## The model library

`g1_model` reads the shipped URDFs and answers the questions a controller keeps
asking: what is the canonical joint order, is this configuration inside the
limits, where is the left foot, and where is the centre of mass. It has no ROS
dependency at runtime, so tests and offline tooling can use it directly.

![The G1 squatting, drawn from the model's forward kinematics](robots/g1/g1_model/docs/g1_squat.gif)

That animation is not a recording. It is drawn frame by frame from the model's
own link poses, with the centre of mass and its ground projection in red, which
means it cannot drift away from what the URDF says. See
[`robots/g1/g1_model/`](robots/g1/g1_model/) for the details.

## What you need

ROS 2 Humble, plus `urdfdom` and `orocos_kdl` for the model library and
`gazebo_ros` for the Gazebo backend. All three ship with a standard desktop
install. MuJoCo is optional and only used for the screenshot above.

## Adding a robot

1. Put the description package under `robots/<name>/description/`.
2. Write `robots/<name>/bringup/<name>_rviz.launch.py`, and any other backend
   you want, each taking a `model` argument.
3. Register it in `config/robots.yaml` with its variants and their URDFs.

The launcher does the rest. There is a commented out `h1` entry in the registry
showing the shape of the thing.

## Rough edges, honestly

- No controllers yet. The robot stands there and looks handsome; nothing walks.
- Gazebo bringup spawns the robot and publishes its state, but there is no
  `ros2_control` stack behind it.
- The G1 MJCF files cannot be opened by the MuJoCo viewer as shipped: their
  `meshdir` does not point at `../meshes`, and `scene_*.xml` redefines the floor
  that the robot file already declares. The description package is upstream
  material and is deliberately left untouched.
- H1 is a placeholder in the registry, not a robot.
