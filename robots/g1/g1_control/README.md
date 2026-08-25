# g1_control

Joint space control for the Unitree G1, built on `ros2_control` and
[`g1_model`](../g1_model).

## JointPdController

`g1_control/JointPdController` claims a position and a velocity state interface
per joint and writes one effort command per joint:

```
effort = kp (q_desired - q) + kd (qd_desired - qd) + gravity_scale * g(q)
```

`g(q)` comes from `g1_model`, which computes the holding torques from the link
masses and the current pose. The command is saturated to the effort limit the
URDF declares for that joint, and a non-finite reading from the hardware
collapses the command to zero.

Targets arrive on `~/joint_targets` as a `sensor_msgs/JointState`. Names are
matched against the controlled joints, positions are clamped to the joint
limits, and joints the message omits keep their present target. On activation
the controller adopts the pose it was activated in, so enabling it commands no
jump.

Gains are set in `config/g1_controllers.yaml`. A single value applies to every
joint, and a list sets them per joint in the order of the `joints` parameter.
Leaving `joints` unset takes every actuated joint of the loaded model in its
canonical order.

## Running it

Against mock hardware, which reports back what is written to it:

```bash
ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=mock
ros2 control list_controllers
ros2 topic echo /joint_states
```

In Gazebo Classic, where the controller manager runs inside the simulator:

```bash
ros2 launch launch/spawn_robot.launch.py robot:=g1 sim:=gazebo_control
```

The G1 description carries no `ros2_control` tags, so both launch files compose
that section around the URDF at launch time and leave the description files
alone.

## Limits

The gravity term treats the pelvis as fixed. Standing on its feet the legs also
carry the contact wrench, so the leg torques are an upper bound rather than the
exact feedforward term; the arms and the waist are exact. `update` allocates,
since the model rebuilds its link poses each cycle, which puts a ceiling on the
rate this controller can hold without dropping cycles.
