#include "g1_model/g1_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <urdf_parser/urdf_parser.h>

namespace g1_model
{
namespace
{

KDL::Frame toKdl(const urdf::Pose & pose)
{
  return KDL::Frame(
    KDL::Rotation::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
    KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

bool isActuated(const urdf::Joint & joint)
{
  return joint.type == urdf::Joint::REVOLUTE || joint.type == urdf::Joint::CONTINUOUS ||
         joint.type == urdf::Joint::PRISMATIC;
}

/// Motion of the child frame relative to the joint frame for the value @p q.
KDL::Frame jointTransform(const urdf::Joint & joint, double q)
{
  const KDL::Vector axis(joint.axis.x, joint.axis.y, joint.axis.z);
  switch (joint.type) {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
      return KDL::Frame(KDL::Rotation::Rot(axis, q));
    case urdf::Joint::PRISMATIC:
      return KDL::Frame(axis * q);
    default:
      return KDL::Frame::Identity();
  }
}

}  // namespace

std::string to_string(Variant variant)
{
  return variant == Variant::Dof29 ? "29dof" : "23dof";
}

G1Model G1Model::fromFile(const std::string & urdf_path)
{
  std::ifstream in(urdf_path);
  if (!in) {
    throw ModelError("cannot open URDF file: " + urdf_path);
  }
  std::ostringstream xml;
  xml << in.rdbuf();
  return fromString(xml.str());
}

G1Model G1Model::fromString(const std::string & urdf_xml)
{
  const auto urdf = urdf::parseURDF(urdf_xml);
  if (!urdf) {
    throw ModelError("failed to parse URDF");
  }
  G1Model model;
  model.build(urdf);
  return model;
}

void G1Model::build(const std::shared_ptr<urdf::ModelInterface> & urdf)
{
  urdf_ = urdf;

  const auto root = urdf->getRoot();
  if (!root) {
    throw ModelError("URDF has no root link");
  }
  root_link_ = root->name;

  // Depth-first pre-order over the tree, so the joint order follows the
  // kinematic structure (left leg, right leg, waist, arms) instead of a hash
  // order.
  std::vector<urdf::LinkConstSharedPtr> stack{root};
  while (!stack.empty()) {
    const auto link = stack.back();
    stack.pop_back();

    if (link->inertial) {
      total_mass_ += link->inertial->mass;
    }

    const auto & joint = link->parent_joint;
    if (joint && isActuated(*joint)) {
      if (!joint->limits) {
        throw ModelError("actuated joint without limits: " + joint->name);
      }
      joint_index_.emplace(joint->name, joint_names_.size());
      joint_names_.push_back(joint->name);
      limits_.push_back(
        JointLimits{joint->limits->lower, joint->limits->upper, joint->limits->velocity,
                    joint->limits->effort});
    }

    // Reversed, so the first child in the URDF is the first one popped.
    for (auto it = link->child_links.rbegin(); it != link->child_links.rend(); ++it) {
      stack.push_back(*it);
    }
  }

  if (joint_names_.size() == 23) {
    variant_ = Variant::Dof23;
  } else if (joint_names_.size() == 29) {
    variant_ = Variant::Dof29;
  } else {
    throw ModelError(
      "unexpected DOF count " + std::to_string(joint_names_.size()) +
      ", expected 23 or 29 for the G1");
  }
}

std::vector<std::string> G1Model::linkNames() const
{
  std::vector<std::string> names;
  names.reserve(urdf_->links_.size());
  for (const auto & entry : urdf_->links_) {
    names.push_back(entry.first);
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::pair<std::string, std::string>> G1Model::treeEdges() const
{
  std::vector<std::pair<std::string, std::string>> edges;
  std::vector<urdf::LinkConstSharedPtr> stack{urdf_->getRoot()};
  while (!stack.empty()) {
    const auto link = stack.back();
    stack.pop_back();
    for (const auto & joint : link->child_joints) {
      edges.emplace_back(joint->parent_link_name, joint->child_link_name);
    }
    for (auto it = link->child_links.rbegin(); it != link->child_links.rend(); ++it) {
      stack.push_back(*it);
    }
  }
  return edges;
}

bool G1Model::hasJoint(const std::string & joint) const noexcept
{
  return joint_index_.find(joint) != joint_index_.end();
}

std::size_t G1Model::indexOf(const std::string & joint) const
{
  const auto it = joint_index_.find(joint);
  if (it == joint_index_.end()) {
    throw ModelError("unknown or non-actuated joint: " + joint);
  }
  return it->second;
}

const JointLimits & G1Model::limits(const std::string & joint) const
{
  return limits_[indexOf(joint)];
}

JointPositions G1Model::midRangeConfiguration() const
{
  JointPositions q(dof());
  for (std::size_t i = 0; i < dof(); ++i) {
    q[i] = 0.5 * (limits_[i].lower + limits_[i].upper);
  }
  return q;
}

void G1Model::checkSize(const JointPositions & q) const
{
  if (q.size() != dof()) {
    throw ModelError(
      "configuration has " + std::to_string(q.size()) + " values, expected " +
      std::to_string(dof()));
  }
}

std::size_t G1Model::firstViolation(const JointPositions & q, double tolerance) const
{
  checkSize(q);
  for (std::size_t i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i]) || q[i] < limits_[i].lower - tolerance ||
        q[i] > limits_[i].upper + tolerance) {
      return i;
    }
  }
  return dof();
}

bool G1Model::withinLimits(const JointPositions & q, double tolerance) const
{
  return firstViolation(q, tolerance) == dof();
}

JointPositions G1Model::clamp(const JointPositions & q) const
{
  checkSize(q);
  JointPositions clamped(q.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    clamped[i] = std::min(std::max(q[i], limits_[i].lower), limits_[i].upper);
  }
  return clamped;
}

std::map<std::string, KDL::Frame> G1Model::linkPoses(const JointPositions & q) const
{
  checkSize(q);

  std::map<std::string, KDL::Frame> poses{{root_link_, KDL::Frame::Identity()}};
  std::vector<urdf::LinkConstSharedPtr> stack{urdf_->getRoot()};
  while (!stack.empty()) {
    const auto link = stack.back();
    stack.pop_back();
    const KDL::Frame & parent_pose = poses.at(link->name);

    for (const auto & joint : link->child_joints) {
      const double value = isActuated(*joint) ? q[joint_index_.at(joint->name)] : 0.0;
      poses.emplace(
        joint->child_link_name,
        parent_pose * toKdl(joint->parent_to_joint_origin_transform) *
          jointTransform(*joint, value));
    }
    for (const auto & child : link->child_links) {
      stack.push_back(child);
    }
  }
  return poses;
}

KDL::Frame G1Model::linkPose(const JointPositions & q, const std::string & link) const
{
  if (!urdf_->getLink(link)) {
    throw ModelError("unknown link: " + link);
  }
  return linkPoses(q).at(link);
}

KDL::Frame G1Model::relativePose(
  const JointPositions & q, const std::string & link, const std::string & reference_link) const
{
  for (const auto & name : {link, reference_link}) {
    if (!urdf_->getLink(name)) {
      throw ModelError("unknown link: " + name);
    }
  }
  const auto poses = linkPoses(q);
  return poses.at(reference_link).Inverse() * poses.at(link);
}

JointPositions G1Model::gravityTorques(
  const JointPositions & q, const KDL::Vector & gravity) const
{
  const auto poses = linkPoses(q);

  // Generalised gravity force at joint j, summed over the links it carries:
  //   Q_j = sum_k m_k * gravity . (axis_j x (com_k - origin_j))
  // The holding torque is its negative.
  JointPositions torques(dof(), 0.0);
  for (const auto & entry : urdf_->links_) {
    const auto & link = entry.second;
    if (!link || !link->inertial || link->inertial->mass <= 0.0) {
      continue;
    }
    const auto pose = poses.find(link->name);
    if (pose == poses.end()) {
      continue;
    }
    const KDL::Vector com = (pose->second * toKdl(link->inertial->origin)).p;
    const KDL::Vector weight = link->inertial->mass * gravity;

    // Walk up to the root, adding this link's weight to every joint above it.
    for (auto ancestor = link; ancestor->parent_joint; ancestor = ancestor->getParent()) {
      const auto & joint = *ancestor->parent_joint;
      const auto index = joint_index_.find(joint.name);
      if (index == joint_index_.end()) {
        continue;  // fixed joint
      }
      const KDL::Frame & joint_frame = poses.at(joint.child_link_name);
      const KDL::Vector axis = joint_frame.M * KDL::Vector(joint.axis.x, joint.axis.y, joint.axis.z);
      const double generalised = joint.type == urdf::Joint::PRISMATIC
        ? dot(weight, axis)
        : dot(weight, axis * (com - joint_frame.p));
      torques[index->second] -= generalised;
    }
  }
  return torques;
}

KDL::Vector G1Model::centerOfMass(const JointPositions & q) const
{
  const auto poses = linkPoses(q);

  KDL::Vector weighted_sum = KDL::Vector::Zero();
  double mass = 0.0;
  for (const auto & entry : urdf_->links_) {
    const auto & link = entry.second;
    if (!link || !link->inertial || link->inertial->mass <= 0.0) {
      continue;
    }
    const auto pose = poses.find(link->name);
    if (pose == poses.end()) {
      continue;  // link is not connected to the root (e.g. a commented-out world link)
    }
    const KDL::Frame com_in_root = pose->second * toKdl(link->inertial->origin);
    weighted_sum = weighted_sum + link->inertial->mass * com_in_root.p;
    mass += link->inertial->mass;
  }
  if (mass <= 0.0) {
    throw ModelError("model has no mass");
  }
  return weighted_sum / mass;
}

}  // namespace g1_model
