// A joint space PD controller with gravity compensation for the Unitree G1.
#ifndef G1_CONTROL__JOINT_PD_CONTROLLER_HPP_
#define G1_CONTROL__JOINT_PD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include "g1_control/pd_law.hpp"
#include "g1_model/g1_model.hpp"

namespace g1_control
{

/// Claims a position and a velocity state interface per joint and writes one
/// effort command per joint.
///
/// Targets arrive on `~/joint_targets` as a JointState message; joints the
/// message does not mention keep their current target. On activation the
/// controller holds the pose it was activated in.
class JointPdController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Reorders an incoming target message into controlled joint order.
  void receiveTargets(const sensor_msgs::msg::JointState::SharedPtr message);

  std::unique_ptr<g1_model::G1Model> model_;
  std::vector<std::string> joints_;
  std::vector<std::size_t> model_index_;  ///< joints_[i] in the model's ordering
  std::vector<Gains> gains_;
  std::vector<double> effort_limits_;
  double gravity_scale_{1.0};

  realtime_tools::RealtimeBuffer<std::vector<double>> targets_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_subscription_;

  g1_model::JointPositions model_positions_;  ///< scratch, reused every cycle
  std::vector<double> gravity_torques_;
};

}  // namespace g1_control

#endif  // G1_CONTROL__JOINT_PD_CONTROLLER_HPP_
