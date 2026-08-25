#include <cmath>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "g1_model/g1_model.hpp"

using g1_model::G1Model;
using g1_model::JointPositions;
using g1_model::ModelError;
using g1_model::Variant;

namespace
{

std::string urdfPath(const std::string & variant)
{
  return std::string(G1_URDF_DIR) + "/g1_" + variant + ".urdf";
}

G1Model load23() { return G1Model::fromFile(urdfPath("23dof")); }
G1Model load29() { return G1Model::fromFile(urdfPath("29dof")); }

constexpr double kEps = 1e-9;

}  // namespace

TEST(Loading, RejectsMissingFile)
{
  EXPECT_THROW(G1Model::fromFile("/nonexistent/g1.urdf"), ModelError);
}

TEST(Loading, RejectsGarbage)
{
  EXPECT_THROW(G1Model::fromString("not xml at all"), ModelError);
}

TEST(Loading, RejectsRobotThatIsNotAG1)
{
  // Well-formed URDF, but nowhere near 23 or 29 DOF.
  const std::string xml =
    R"(<robot name="stub">
         <link name="base"/>
         <link name="tip"/>
         <joint name="j" type="revolute">
           <parent link="base"/><child link="tip"/>
           <axis xyz="0 0 1"/>
           <limit lower="-1" upper="1" effort="1" velocity="1"/>
         </joint>
       </robot>)";
  EXPECT_THROW(G1Model::fromString(xml), ModelError);
}

TEST(Structure, DofCountsAndVariants)
{
  EXPECT_EQ(load23().dof(), 23u);
  EXPECT_EQ(load23().variant(), Variant::Dof23);
  EXPECT_EQ(load29().dof(), 29u);
  EXPECT_EQ(load29().variant(), Variant::Dof29);
  EXPECT_EQ(g1_model::to_string(Variant::Dof29), "29dof");
}

TEST(Structure, RootIsPelvis)
{
  EXPECT_EQ(load23().rootLink(), "pelvis");
  EXPECT_EQ(load29().rootLink(), "pelvis");
}

TEST(Structure, JointOrderFollowsKinematicTree)
{
  const auto model = load23();
  const std::vector<std::string> expected_legs{
    "left_hip_pitch_joint",     "left_hip_roll_joint",  "left_hip_yaw_joint",
    "left_knee_joint",          "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint",    "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint",         "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint"};

  const auto & names = model.jointNames();
  ASSERT_GE(names.size(), expected_legs.size());
  for (std::size_t i = 0; i < expected_legs.size(); ++i) {
    EXPECT_EQ(names[i], expected_legs[i]) << "at index " << i;
  }
}

TEST(Structure, JointNamesAreUniqueAndIndexable)
{
  const auto model = load29();
  std::set<std::string> unique(model.jointNames().begin(), model.jointNames().end());
  EXPECT_EQ(unique.size(), model.dof());

  for (std::size_t i = 0; i < model.dof(); ++i) {
    EXPECT_TRUE(model.hasJoint(model.jointNames()[i]));
    EXPECT_EQ(model.indexOf(model.jointNames()[i]), i);
  }
  EXPECT_FALSE(model.hasJoint("waist_yaw_fixed_joint"));  // fixed joints are not actuated
  EXPECT_THROW(model.indexOf("no_such_joint"), ModelError);
  EXPECT_THROW(model.limits("no_such_joint"), ModelError);
}

TEST(Structure, TwentyNineDofIsASupersetOfTwentyThree)
{
  const auto small = load23();
  const auto large = load29();
  for (const auto & joint : small.jointNames()) {
    EXPECT_TRUE(large.hasJoint(joint)) << joint << " missing from the 29-DOF model";
  }
  for (const auto & joint : {"waist_roll_joint", "waist_pitch_joint", "left_wrist_pitch_joint",
                             "left_wrist_yaw_joint", "right_wrist_pitch_joint",
                             "right_wrist_yaw_joint"}) {
    EXPECT_TRUE(large.hasJoint(joint)) << joint;
    EXPECT_FALSE(small.hasJoint(joint)) << joint;
  }
}

TEST(Limits, AreWellFormed)
{
  for (const auto & model : {load23(), load29()}) {
    for (std::size_t i = 0; i < model.dof(); ++i) {
      const auto & limit = model.limits()[i];
      const auto & name = model.jointNames()[i];
      EXPECT_LT(limit.lower, limit.upper) << name;
      EXPECT_GT(limit.velocity, 0.0) << name;
      EXPECT_GT(limit.effort, 0.0) << name;
    }
  }
}

TEST(Limits, LegJointsAreMirrored)
{
  const auto model = load29();
  // Pitch joints mirror directly; roll/yaw axes flip sign between the two legs.
  const auto & left_knee = model.limits("left_knee_joint");
  const auto & right_knee = model.limits("right_knee_joint");
  EXPECT_NEAR(left_knee.lower, right_knee.lower, kEps);
  EXPECT_NEAR(left_knee.upper, right_knee.upper, kEps);

  const auto & left_roll = model.limits("left_hip_roll_joint");
  const auto & right_roll = model.limits("right_hip_roll_joint");
  EXPECT_NEAR(left_roll.lower, -right_roll.upper, kEps);
  EXPECT_NEAR(left_roll.upper, -right_roll.lower, kEps);
}

TEST(Configuration, ZeroPoseIsReachable)
{
  const auto model = load29();
  EXPECT_TRUE(model.withinLimits(model.zeroConfiguration()));
  EXPECT_TRUE(model.withinLimits(model.midRangeConfiguration()));
  EXPECT_EQ(model.firstViolation(model.zeroConfiguration()), model.dof());
}

TEST(Configuration, DetectsAndClampsViolations)
{
  const auto model = load23();
  auto q = model.zeroConfiguration();
  const auto knee = model.indexOf("left_knee_joint");
  q[knee] = model.limits()[knee].upper + 0.5;

  EXPECT_FALSE(model.withinLimits(q));
  EXPECT_EQ(model.firstViolation(q), knee);
  EXPECT_TRUE(model.withinLimits(q, 0.6));  // tolerance swallows the overshoot

  const auto clamped = model.clamp(q);
  EXPECT_TRUE(model.withinLimits(clamped));
  EXPECT_NEAR(clamped[knee], model.limits()[knee].upper, kEps);
  for (std::size_t i = 0; i < model.dof(); ++i) {
    if (i != knee) {
      EXPECT_NEAR(clamped[i], q[i], kEps) << model.jointNames()[i];
    }
  }
}

TEST(Configuration, RejectsNaNAndWrongSize)
{
  const auto model = load23();
  auto q = model.zeroConfiguration();
  q[0] = std::nan("");
  EXPECT_FALSE(model.withinLimits(q));

  const JointPositions too_short(model.dof() - 1, 0.0);
  EXPECT_THROW(model.withinLimits(too_short), ModelError);
  EXPECT_THROW(model.clamp(too_short), ModelError);
  EXPECT_THROW(model.linkPose(too_short, "left_ankle_roll_link"), ModelError);
}

TEST(Kinematics, RootPoseIsIdentity)
{
  const auto model = load23();
  const auto pose = model.linkPose(model.zeroConfiguration(), "pelvis");
  EXPECT_TRUE(KDL::Equal(pose, KDL::Frame::Identity(), 1e-12));
}

TEST(Kinematics, RejectsUnknownLinks)
{
  const auto model = load23();
  const auto q = model.zeroConfiguration();
  EXPECT_THROW(model.linkPose(q, "left_hand_link"), ModelError);
  EXPECT_THROW(model.relativePose(q, "pelvis", "nowhere"), ModelError);
}

TEST(Kinematics, FeetAreMirroredAndBelowThePelvis)
{
  const auto model = load29();
  const auto q = model.zeroConfiguration();
  const auto left = model.linkPose(q, "left_ankle_roll_link");
  const auto right = model.linkPose(q, "right_ankle_roll_link");

  EXPECT_NEAR(left.p.x(), right.p.x(), 1e-9);
  EXPECT_NEAR(left.p.z(), right.p.z(), 1e-9);
  EXPECT_NEAR(left.p.y(), -right.p.y(), 1e-9);
  EXPECT_GT(left.p.y(), 0.0);                       // +Y is the robot's left
  EXPECT_LT(left.p.z(), -0.5);                      // standing height of the leg chain
  EXPECT_GT(left.p.z(), -1.0);
}

TEST(Kinematics, KneeFlexionRaisesTheFoot)
{
  const auto model = load23();
  const auto straight = model.linkPose(model.zeroConfiguration(), "left_ankle_roll_link");

  auto q = model.zeroConfiguration();
  q[model.indexOf("left_knee_joint")] = 1.0;  // rad
  const auto bent = model.linkPose(q, "left_ankle_roll_link");

  EXPECT_GT(bent.p.z(), straight.p.z() + 0.05);
  EXPECT_NEAR(model.linkPose(q, "right_ankle_roll_link").p.z(), straight.p.z(), 1e-12)
    << "the other leg must not move";
}

TEST(Kinematics, HipYawRotatesTheFootByTheCommandedAngle)
{
  const auto model = load23();
  auto q = model.zeroConfiguration();
  q[model.indexOf("left_hip_yaw_joint")] = 0.3;

  // The hip yaw axis is not world-aligned (the hip pitch/roll origins are
  // tilted), so check the rotation magnitude rather than a single RPY angle.
  const auto neutral = model.linkPose(model.zeroConfiguration(), "left_ankle_roll_link");
  const auto rotated = model.linkPose(q, "left_ankle_roll_link");
  const KDL::Rotation delta = neutral.M.Inverse() * rotated.M;
  KDL::Vector axis;
  EXPECT_NEAR(delta.GetRotAngle(axis), 0.3, 1e-9);
  EXPECT_GT(std::abs(axis.z()), 0.9) << "hip yaw is still mostly a vertical axis";
}

TEST(Kinematics, RelativePoseIsConsistent)
{
  const auto model = load29();
  auto q = model.midRangeConfiguration();

  const auto direct = model.linkPose(q, "left_elbow_link");
  const auto via_root = model.relativePose(q, "left_elbow_link", "pelvis");
  EXPECT_TRUE(KDL::Equal(direct, via_root, 1e-12));

  const auto l_to_r = model.relativePose(q, "left_elbow_link", "right_elbow_link");
  const auto r_to_l = model.relativePose(q, "right_elbow_link", "left_elbow_link");
  EXPECT_TRUE(KDL::Equal(l_to_r * r_to_l, KDL::Frame::Identity(), 1e-12));
}

TEST(Kinematics, FixedFramesFollowTheirParent)
{
  const auto model = load29();
  auto q = model.zeroConfiguration();
  q[model.indexOf("waist_yaw_joint")] = 0.4;

  // The IMU inside the torso is rigidly attached, so it must yaw with it.
  const auto imu = model.relativePose(q, "imu_in_torso", "torso_link");
  const auto imu_zero = model.relativePose(model.zeroConfiguration(), "imu_in_torso",
                                           "torso_link");
  EXPECT_TRUE(KDL::Equal(imu, imu_zero, 1e-12));

  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  model.linkPose(q, "imu_in_torso").M.GetRPY(roll, pitch, yaw);
  EXPECT_NEAR(yaw, 0.4, 1e-9);
}

TEST(Kinematics, LinkPosesCoverTheWholeTree)
{
  const auto model = load29();
  const auto q = model.midRangeConfiguration();
  const auto poses = model.linkPoses(q);

  EXPECT_EQ(poses.size(), model.linkNames().size());
  for (const auto & link : model.linkNames()) {
    ASSERT_EQ(poses.count(link), 1u) << link;
    EXPECT_TRUE(KDL::Equal(poses.at(link), model.linkPose(q, link), 1e-12)) << link;
  }
}

TEST(Structure, TreeEdgesConnectEveryNonRootLink)
{
  const auto model = load29();
  const auto edges = model.treeEdges();
  EXPECT_EQ(edges.size(), model.linkNames().size() - 1);  // a tree, so |E| = |V| - 1

  std::set<std::string> children;
  for (const auto & edge : edges) {
    EXPECT_TRUE(model.linkPoses(model.zeroConfiguration()).count(edge.first)) << edge.first;
    EXPECT_TRUE(children.insert(edge.second).second) << edge.second << " has two parents";
  }
  EXPECT_EQ(children.count(model.rootLink()), 0u) << "the root has no parent";
}

TEST(Inertia, TotalMassIsPlausible)
{
  const auto mass_23 = load23().totalMass();
  const auto mass_29 = load29().totalMass();
  EXPECT_GT(mass_23, 20.0);
  EXPECT_LT(mass_23, 60.0);
  EXPECT_GT(mass_29, mass_23);  // extra waist and wrist actuators
}

TEST(Inertia, CenterOfMassIsCenteredAtZeroPose)
{
  const auto model = load29();
  const auto com = model.centerOfMass(model.zeroConfiguration());
  EXPECT_NEAR(com.y(), 0.0, 1e-3) << "the zero pose is laterally symmetric";
  EXPECT_LT(std::abs(com.x()), 0.1);
  EXPECT_LT(std::abs(com.z()), 0.4);
}

TEST(Inertia, CenterOfMassShiftsWithTheLegs)
{
  const auto model = load29();
  auto q = model.zeroConfiguration();
  q[model.indexOf("left_hip_roll_joint")] = 0.3;

  const auto neutral = model.centerOfMass(model.zeroConfiguration());
  const auto shifted = model.centerOfMass(q);
  EXPECT_GT(shifted.y(), neutral.y() + 1e-3) << "swinging the left leg out moves mass to +Y";
  EXPECT_THROW(model.centerOfMass(JointPositions(3, 0.0)), ModelError);
}
