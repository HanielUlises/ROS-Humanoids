#include "g1_control/joint_pd_controller.hpp"

#include <algorithm>
#include <limits>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace g1_control
{
namespace
{

/// Reads a scalar or a per joint list of gains into a vector of `count` values.
std::vector<double> gainVector(
  const std::vector<double> & configured, std::size_t count, const std::string & name)
{
  if (configured.size() == 1) {
    return std::vector<double>(count, configured.front());
  }
  if (configured.size() != count) {
    throw std::runtime_error(
      name + " must hold either one value or one per joint, got " +
      std::to_string(configured.size()) + " for " + std::to_string(count) + " joints");
  }
  return configured;
}

}  // namespace

controller_interface::CallbackReturn JointPdController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<double>>("kp", {60.0});
    auto_declare<std::vector<double>>("kd", {3.0});
    auto_declare<double>("gravity_scale", 1.0);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "cannot declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPdController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto urdf = get_node()->get_parameter("robot_description").as_string();
  if (urdf.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "the robot_description parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    model_ = std::make_unique<g1_model::G1Model>(g1_model::G1Model::fromString(urdf));

    const auto configured_joints = get_node()->get_parameter("joints");
    joints_ = configured_joints.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY
      ? configured_joints.as_string_array()
      : std::vector<std::string>{};
    if (joints_.empty()) {
      joints_ = model_->jointNames();
    }

    model_index_.clear();
    effort_limits_.clear();
    for (const auto & joint : joints_) {
      model_index_.push_back(model_->indexOf(joint));
      effort_limits_.push_back(model_->limits(joint).effort);
    }

    const auto kp = gainVector(get_node()->get_parameter("kp").as_double_array(), joints_.size(), "kp");
    const auto kd = gainVector(get_node()->get_parameter("kd").as_double_array(), joints_.size(), "kd");
    gains_.clear();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      gains_.push_back(Gains{kp[i], kd[i]});
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "cannot configure: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  gravity_scale_ = get_node()->get_parameter("gravity_scale").as_double();
  model_positions_ = model_->zeroConfiguration();
  gravity_torques_.assign(joints_.size(), 0.0);
  targets_.writeFromNonRT(std::vector<double>(joints_.size(), 0.0));

  target_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~/joint_targets", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) { receiveTargets(message); });

  RCLCPP_INFO(
    get_node()->get_logger(), "configured for the %s G1, %zu joints, gravity scale %.2f",
    g1_model::to_string(model_->variant()).c_str(), joints_.size(), gravity_scale_);
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointPdController::receiveTargets(const sensor_msgs::msg::JointState::SharedPtr message)
{
  auto targets = *targets_.readFromNonRT();
  for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
    const auto found = std::find(joints_.begin(), joints_.end(), message->name[i]);
    if (found == joints_.end()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "ignoring target for uncontrolled joint '%s'", message->name[i].c_str());
      continue;
    }
    const auto index = static_cast<std::size_t>(std::distance(joints_.begin(), found));
    const auto & limits = model_->limits(joints_[index]);
    targets[index] = std::clamp(message->position[i], limits.lower, limits.upper);
  }
  targets_.writeFromNonRT(targets);
}

controller_interface::InterfaceConfiguration
JointPdController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
JointPdController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::CallbackReturn JointPdController::on_activate(const rclcpp_lifecycle::State &)
{
  // Hold the pose the robot is activated in, so that enabling the controller
  // does not command a jump.
  std::vector<double> targets(joints_.size(), 0.0);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    targets[i] = state_interfaces_[2 * i].get_value();
  }
  targets_.writeFromNonRT(targets);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPdController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & command : command_interfaces_) {
    command.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPdController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto & targets = *targets_.readFromRT();

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    model_positions_[model_index_[i]] = state_interfaces_[2 * i].get_value();
  }

  if (gravity_scale_ != 0.0) {
    const auto torques = model_->gravityTorques(model_positions_);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      gravity_torques_[i] = gravity_scale_ * torques[model_index_[i]];
    }
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const JointState state{
      state_interfaces_[2 * i].get_value(), state_interfaces_[2 * i + 1].get_value(), targets[i],
      0.0};
    command_interfaces_[i].set_value(
      effortCommand(gains_[i], state, gravity_torques_[i], effort_limits_[i]));
  }
  return controller_interface::return_type::OK;
}

}  // namespace g1_control

PLUGINLIB_EXPORT_CLASS(g1_control::JointPdController, controller_interface::ControllerInterface)
