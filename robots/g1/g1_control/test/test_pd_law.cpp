#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "g1_control/pd_law.hpp"
#include "g1_model/g1_model.hpp"

using g1_control::effortCommand;
using g1_control::Gains;
using g1_control::JointState;

namespace
{

g1_model::G1Model loadG1()
{
  return g1_model::G1Model::fromFile(std::string(G1_URDF_DIR) + "/g1_29dof.urdf");
}

constexpr double kNoLimit = 0.0;

}  // namespace

TEST(EffortCommand, ZeroErrorLeavesTheGravityTerm)
{
  const JointState state{0.4, 0.0, 0.4, 0.0};
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, state, 7.5, kNoLimit), 7.5);
}

TEST(EffortCommand, PullsTowardsTheTarget)
{
  const JointState below{0.0, 0.0, 0.5, 0.0};
  const JointState above{0.5, 0.0, 0.0, 0.0};
  EXPECT_GT(effortCommand(Gains{100.0, 5.0}, below, 0.0, kNoLimit), 0.0);
  EXPECT_LT(effortCommand(Gains{100.0, 5.0}, above, 0.0, kNoLimit), 0.0);
}

TEST(EffortCommand, DampsMotion)
{
  const JointState moving{0.0, 2.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, moving, 0.0, kNoLimit), -10.0);

  const JointState tracking{0.0, 2.0, 0.0, 2.0};
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, tracking, 0.0, kNoLimit), 0.0);
}

TEST(EffortCommand, SaturatesAtTheEffortLimit)
{
  const JointState far{0.0, 0.0, 3.0, 0.0};
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, far, 0.0, 25.0), 25.0);

  const JointState far_below{3.0, 0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, far_below, 0.0, 25.0), -25.0);
}

TEST(EffortCommand, RefusesToActOnBadReadings)
{
  const JointState nan_position{std::nan(""), 0.0, 0.0, 0.0};
  const JointState nan_velocity{0.0, std::nan(""), 0.0, 0.0};
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, nan_position, 4.0, 50.0), 0.0);
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, nan_velocity, 4.0, 50.0), 0.0);

  const JointState fine{0.0, 0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(effortCommand(Gains{100.0, 5.0}, fine, std::nan(""), 50.0), 0.0);
}

TEST(EffortCommand, StaysWithinTheModelEffortLimits)
{
  const auto model = loadG1();
  const auto q = model.midRangeConfiguration();
  const auto gravity = model.gravityTorques(q);

  for (std::size_t i = 0; i < model.dof(); ++i) {
    const auto & limits = model.limits()[i];
    // A target at the far end of the range, which a plain PD term would badly
    // overshoot on the low effort wrist joints.
    const JointState state{q[i], 0.0, limits.upper, 0.0};
    const double effort = effortCommand(Gains{500.0, 10.0}, state, gravity[i], limits.effort);
    EXPECT_LE(std::abs(effort), limits.effort + 1e-9) << model.jointNames()[i];
  }
}

TEST(GravityFeedforward, HoldsTheArmsUp)
{
  const auto model = loadG1();
  auto q = model.zeroConfiguration();
  q[model.indexOf("left_shoulder_pitch_joint")] = -1.5;  // arm out in front
  const auto gravity = model.gravityTorques(q);

  const std::size_t shoulder = model.indexOf("left_shoulder_pitch_joint");
  const JointState resting{q[shoulder], 0.0, q[shoulder], 0.0};
  const double effort =
    effortCommand(Gains{80.0, 4.0}, resting, gravity[shoulder], model.limits()[shoulder].effort);

  EXPECT_GT(std::abs(effort), 1.0) << "an outstretched arm needs holding torque";
  EXPECT_LE(std::abs(effort), model.limits()[shoulder].effort);
}
