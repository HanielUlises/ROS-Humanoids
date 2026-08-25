# g1_model

C++ model of the Unitree G1, built directly from the URDFs in
[`g1_description`](../description). Header-light, no ROS node or middleware
involved — it is a plain library that controllers, state publishers and tests
can share.

![The G1 squatting, drawn from the model's forward kinematics](docs/g1_squat.gif)

*Every line above is drawn from `linkPoses()`; the red circle is `centerOfMass()`
and the red dot on the ground is its projection. Nothing is hard-coded — the
figure is exactly what the shipped URDF says.*

It covers both variants (`g1_23dof.urdf`, `g1_29dof.urdf`) and provides:

- a **canonical joint order** taken from the kinematic tree (left leg, right
  leg, waist, arms) so that joint vectors mean the same thing everywhere;
- **joint limits** (position, velocity, effort) with `withinLimits`,
  `firstViolation` and `clamp`;
- **forward kinematics** for any link, absolute or relative to another link;
- **mass and whole-body centre of mass** in the pelvis frame.

## Use

```cpp
#include "g1_model/g1_model.hpp"

const auto model = g1_model::G1Model::fromFile(urdf_path);   // or fromString(xml)

auto q = model.zeroConfiguration();
q[model.indexOf("left_knee_joint")] = 0.6;

if (!model.withinLimits(q)) {
  q = model.clamp(q);
}

const KDL::Frame foot = model.linkPose(q, "left_ankle_roll_link");
const KDL::Vector com = model.centerOfMass(q);
```

Errors (missing file, malformed URDF, unknown joint or link, wrongly sized
configuration) are reported as `g1_model::ModelError`.

## Build and test

```bash
colcon build --packages-select g1_model
colcon test --packages-select g1_model --event-handlers console_direct+
```

Or standalone, without a workspace:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build build
./build/test_g1_model
```

The tests read the URDFs straight out of `../description/urdf`, so they fail if
the shipped description changes shape (DOF count, joint names, mirrored limits).

## Regenerating the animation

The GIF is produced from the model itself, so it stays honest if the URDF
changes:

```bash
ros2 run g1_model g1_skeleton_dump robots/g1/description/urdf/g1_29dof.urdf 48 \
  | tools/render_skeleton.py docs/g1_squat.gif
```

`g1_skeleton_dump` sweeps a squat-and-swing motion (clamped to the joint
limits) and prints the kinematic-tree edges, every link origin and the centre of
mass per frame; `tools/render_skeleton.py` draws the front and side views with
Pillow. The renderer drops each frame onto the ground line, since the model is
expressed in the pelvis frame.

## Inspecting a URDF

```bash
ros2 run g1_model g1_info robots/g1/description/urdf/g1_29dof.urdf
```

prints the joint table, the foot poses and the centre of mass at the zero pose
(35.115 kg, feet at ±0.119 m, 0.757 m below the pelvis for the 29-DOF model).
