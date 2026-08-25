# ROS-Humanoids

A ROS 2 workspace for humanoid robot descriptions and bringup. Robots are
declared in a single registry, and one launch entry point resolves that registry
and dispatches to the bringup files of the robot that was asked for. The
workspace holds the Unitree G1 and the Unitree H1, a C++ library that turns the
G1 description into joint limits, forward kinematics, a centre of mass and
gravity torques, and a `ros2_control` controller built on it.

| Unitree G1 | Unitree H1 |
| --- | --- |
| ![The Unitree G1 in MuJoCo](docs/g1_mujoco.png) | ![The Unitree H1 in MuJoCo](docs/h1_mujoco.png) |

## Requirements

ROS 2 Humble with `gazebo_ros`, `ros2_control`, `gazebo_ros2_control`,
`robot_state_publisher`, `joint_state_publisher_gui` and `rviz2`. The model
library additionally needs `urdfdom` and `orocos_kdl`. MuJoCo and Pillow are
optional and used only by the renderers in `tools/`.

## Quick start

```bash
colcon build
source install/setup.bash

ros2 launch launch/spawn_robot.launch.py robot:=g1
ros2 launch launch/spawn_robot.launch.py robot:=g1 model:=29dof
ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo z:=1.0
ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo_control
ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=mock
ros2 launch launch/spawn_robot.launch.py robot:=h1 sim:=gazebo
```

The `gazebo_control` backend runs the robot under `ros2_control`, and `mock`
runs the same stack against mock hardware without a simulator.

`robot:`, `sim:` and `model:` are validated against `config/robots.yaml`, and an
unrecognised value reports the alternatives that the registry offers.

## Repository layout

| Path | Contents |
| --- | --- |
| `config/robots.yaml` | the robot registry: variants, descriptions and bringup directories |
| `launch/spawn_robot.launch.py` | the entry point that resolves the registry and includes a robot's bringup |
| `launch/bringup_common.py` | description lookup and `ros2_control` composition shared by the bringup files |
| `robots/<robot>/description/` | URDF, MJCF, meshes and viewer configuration |
| `robots/<robot>/bringup/` | the launch file per backend for that robot |
| `robots/g1/g1_model/` | the C++ model library for the G1 |
| `robots/g1/g1_control/` | the `ros2_control` controller for the G1 |
| `tools/` | offscreen MuJoCo renderer used for the screenshots |
| `docs/` | generated images |

## Supported robots

| Robot | Variants | Actuated joints | Mass | Backends |
| --- | --- | --- | --- | --- |
| Unitree G1 | `23dof`, `29dof` | 23, 29 | 34.1 kg, 35.1 kg | RViz, Gazebo Classic, Gazebo under `ros2_control`, mock hardware |
| Unitree H1 | `19dof` | 19 | 59.3 kg | RViz, Gazebo Classic |

The G1 `23dof` build has a fixed waist and roll-only wrists; the `29dof` build
adds waist roll and pitch and two further joints per wrist. The H1 carries five
joints per leg, four per arm and a torso yaw joint.

## Model library

[`robots/g1/g1_model/`](robots/g1/g1_model/) reads the shipped URDFs and exposes
a canonical joint order, joint limits with clamping, forward kinematics for any
link, and the whole-body centre of mass. It has no runtime ROS dependency, so
offline tooling and tests can link against it directly. Its test suite runs
against the description files themselves and fails if their shape changes.

![The G1 squatting, drawn from the model's forward kinematics](robots/g1/g1_model/docs/g1_squat.gif)

The animation is drawn frame by frame from the model's link poses, with the
centre of mass and its ground projection marked in red.

## Control

[`robots/g1/g1_control/`](robots/g1/g1_control/) provides
`g1_control/JointPdController`, a `ros2_control` controller that reads position
and velocity per joint and writes effort:

```
effort = kp (q_desired - q) + kd (qd_desired - qd) + gravity_scale * g(q)
```

The gravity term comes from `g1_model` and treats the pelvis as fixed, so the
arm and waist torques are exact while the leg torques bound the standing case.
Every command is saturated to the effort limit the URDF declares. Targets arrive
on `~/joint_targets` as a `JointState`, and the controller holds the pose it was
activated in until one does.

The descriptions carry no `ros2_control` tags, so the launch files compose that
section around the URDF at launch time and the description files stay as they
were vendored.

## Adding a robot

1. Place the description package under `robots/<name>/description/`.
2. Add `robots/<name>/bringup/<name>_rviz.launch.py`, and any further backend,
   each accepting a `model` argument.
3. Register the robot in `config/robots.yaml` with its variants and their URDFs.

## Notes

The model library and the controller are written against the G1 and read its
joint set directly, so they refuse a description of another shape.

The G1 MJCF files cannot be opened by the MuJoCo viewer as shipped: their
`meshdir` does not resolve to `../meshes`, and `scene_*.xml` redefines assets
that the robot file already declares. The renderer in `tools/` works around both
while loading, and the G1 description stays as it was vendored.

## Licensing

The robot descriptions come from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)
under the BSD 3-Clause licence. See `robots/h1/description/README.md` for the
changes made to the H1 package while vendoring it.
