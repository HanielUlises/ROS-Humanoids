// Kinematic / inertial model of the Unitree G1, built from the URDFs shipped
// in g1_description.
#ifndef G1_MODEL__G1_MODEL_HPP_
#define G1_MODEL__G1_MODEL_HPP_

#include <array>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <kdl/frames.hpp>
#include <urdf_model/model.h>

namespace g1_model
{

/// Which G1 build the loaded URDF describes.
enum class Variant
{
  Dof23,  ///< fixed waist roll/pitch, roll-only wrists
  Dof29,  ///< full waist and 3-DOF wrists
};

std::string to_string(Variant variant);

/// Position/velocity/effort bounds of a single actuated joint.
struct JointLimits
{
  double lower{0.0};     ///< [rad]
  double upper{0.0};     ///< [rad]
  double velocity{0.0};  ///< [rad/s]
  double effort{0.0};    ///< [Nm]
};

/// Thrown for malformed URDFs, unknown joints/links and wrongly sized configurations.
class ModelError : public std::runtime_error
{
public:
  explicit ModelError(const std::string & what) : std::runtime_error(what) {}
};

/// A joint-space configuration, ordered as G1Model::jointNames().
using JointPositions = std::vector<double>;

/// Read-only model of the G1: joint ordering, limits, forward kinematics and
/// whole-body centre of mass.
///
/// The model is immutable once loaded, so a single instance can be shared by
/// several readers (controllers, state publishers, tests).
class G1Model
{
public:
  /// Load from a URDF file on disk.
  static G1Model fromFile(const std::string & urdf_path);

  /// Load from a URDF already held in memory.
  static G1Model fromString(const std::string & urdf_xml);

  Variant variant() const noexcept { return variant_; }

  /// Number of actuated (revolute) joints: 23 or 29.
  std::size_t dof() const noexcept { return joint_names_.size(); }

  /// Actuated joints in kinematic-tree order: left leg, right leg, waist, arms.
  const std::vector<std::string> & jointNames() const noexcept { return joint_names_; }

  /// Name of the root link (`pelvis` for both variants).
  const std::string & rootLink() const noexcept { return root_link_; }

  /// Every link in the model, including fixed sensor and cosmetic frames.
  std::vector<std::string> linkNames() const;

  bool hasJoint(const std::string & joint) const noexcept;

  /// Index of @p joint inside jointNames().
  /// @throws ModelError if the joint is unknown or not actuated.
  std::size_t indexOf(const std::string & joint) const;

  /// @throws ModelError if the joint is unknown or not actuated.
  const JointLimits & limits(const std::string & joint) const;

  /// Limits for every actuated joint, in jointNames() order.
  const std::vector<JointLimits> & limits() const noexcept { return limits_; }

  /// Configuration with every joint at zero (the URDF nominal pose).
  JointPositions zeroConfiguration() const { return JointPositions(dof(), 0.0); }

  /// Configuration at the midpoint of every joint range.
  JointPositions midRangeConfiguration() const;

  /// True if every joint of @p q lies within its limits, up to @p tolerance.
  /// @throws ModelError if q.size() != dof().
  bool withinLimits(const JointPositions & q, double tolerance = 0.0) const;

  /// Index of the first joint of @p q outside its limits, or dof() if none is.
  /// @throws ModelError if q.size() != dof().
  std::size_t firstViolation(const JointPositions & q, double tolerance = 0.0) const;

  /// @p q with every joint saturated to its limits.
  /// @throws ModelError if q.size() != dof().
  JointPositions clamp(const JointPositions & q) const;

  /// Pose of @p link relative to rootLink() for the configuration @p q.
  /// @throws ModelError if the link is unknown or q.size() != dof().
  KDL::Frame linkPose(const JointPositions & q, const std::string & link) const;

  /// Pose of @p link relative to @p reference_link.
  /// @throws ModelError if either link is unknown or q.size() != dof().
  KDL::Frame relativePose(
    const JointPositions & q, const std::string & link,
    const std::string & reference_link) const;

  /// Summed mass of all links [kg].
  double totalMass() const noexcept { return total_mass_; }

  /// Whole-body centre of mass in the rootLink() frame [m].
  /// @throws ModelError if q.size() != dof().
  KDL::Vector centerOfMass(const JointPositions & q) const;

  /// Pose of every link in the rootLink() frame, keyed by link name. Cheaper
  /// than calling linkPose() repeatedly, and handy for drawing the whole robot.
  /// @throws ModelError if q.size() != dof().
  std::map<std::string, KDL::Frame> linkPoses(const JointPositions & q) const;

  /// Parent/child link pairs of every joint, i.e. the edges of the kinematic
  /// tree, in depth-first order from the root.
  std::vector<std::pair<std::string, std::string>> treeEdges() const;

private:
  G1Model() = default;

  void build(const std::shared_ptr<urdf::ModelInterface> & urdf);
  void checkSize(const JointPositions & q) const;

  std::shared_ptr<urdf::ModelInterface> urdf_;
  Variant variant_{Variant::Dof23};
  std::string root_link_;
  std::vector<std::string> joint_names_;
  std::vector<JointLimits> limits_;
  std::map<std::string, std::size_t> joint_index_;
  double total_mass_{0.0};
};

}  // namespace g1_model

#endif  // G1_MODEL__G1_MODEL_HPP_
