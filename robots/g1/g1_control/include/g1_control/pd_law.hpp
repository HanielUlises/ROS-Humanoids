// The per joint control law, kept free of ROS so that it can be unit tested.
#ifndef G1_CONTROL__PD_LAW_HPP_
#define G1_CONTROL__PD_LAW_HPP_

#include <algorithm>
#include <cmath>

namespace g1_control
{

/// Proportional and derivative gains of a single joint.
struct Gains
{
  double kp{0.0};  ///< [Nm/rad]
  double kd{0.0};  ///< [Nm s/rad]
};

/// Measured and requested state of a single joint.
struct JointState
{
  double position{0.0};           ///< [rad]
  double velocity{0.0};           ///< [rad/s]
  double desired_position{0.0};   ///< [rad]
  double desired_velocity{0.0};   ///< [rad/s]
};

/// Effort command for one joint: PD tracking plus a gravity feedforward term,
/// saturated to the joint's effort limit.
///
/// A non-finite measurement or gravity term collapses the command to zero, so a
/// bad reading from the hardware cannot be amplified into a large effort.
inline double effortCommand(
  const Gains & gains, const JointState & state, double gravity_torque, double effort_limit)
{
  if (!std::isfinite(state.position) || !std::isfinite(state.velocity) ||
      !std::isfinite(state.desired_position) || !std::isfinite(gravity_torque)) {
    return 0.0;
  }

  const double effort = gains.kp * (state.desired_position - state.position) +
                        gains.kd * (state.desired_velocity - state.velocity) + gravity_torque;

  const double limit = std::abs(effort_limit);
  return limit > 0.0 ? std::clamp(effort, -limit, limit) : effort;
}

}  // namespace g1_control

#endif  // G1_CONTROL__PD_LAW_HPP_
