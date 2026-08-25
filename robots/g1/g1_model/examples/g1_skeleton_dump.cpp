// Dumps a short animated motion of the G1 as plain text, for the renderer in
// tools/render_skeleton.py.
//
//   ros2 run g1_model g1_skeleton_dump <path-to-g1_XXdof.urdf> [frames]
//
// Output:
//   edge <parent-link> <child-link>          (once, before any frame)
//   frame <index>
//   link <name> <x> <y> <z>                  (per link, per frame)
//   com <x> <y> <z>                          (per frame)

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "g1_model/g1_model.hpp"

namespace
{

/// A looping squat with counter-swinging arms and a waist twist: enough joints
/// move that the drawing exercises both legs, the waist and both arms.
g1_model::JointPositions motionAt(const g1_model::G1Model & model, double phase)
{
  auto q = model.zeroConfiguration();
  const double squat = 0.5 * (1.0 - std::cos(phase));         // 0 .. 1 .. 0
  const double swing = std::sin(phase);                       // -1 .. 1

  const auto set = [&](const std::string & joint, double value) {
    if (model.hasJoint(joint)) {
      q[model.indexOf(joint)] = value;
    }
  };

  for (const std::string side : {"left", "right"}) {
    set(side + "_hip_pitch_joint", -0.9 * squat);
    set(side + "_knee_joint", 1.8 * squat);
    set(side + "_ankle_pitch_joint", -0.85 * squat);
  }

  set("left_shoulder_pitch_joint", -0.9 * swing);
  set("right_shoulder_pitch_joint", 0.9 * swing);
  set("left_shoulder_roll_joint", 0.15 + 0.35 * squat);
  set("right_shoulder_roll_joint", -0.15 - 0.35 * squat);
  set("left_elbow_joint", 0.6 * squat);
  set("right_elbow_joint", 0.6 * squat);
  set("waist_yaw_joint", 0.25 * swing);

  return model.clamp(q);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: g1_skeleton_dump <urdf-file> [frames]\n";
    return 2;
  }
  const int frames = argc == 3 ? std::atoi(argv[2]) : 48;
  if (frames <= 0) {
    std::cerr << "g1_skeleton_dump: frames must be positive\n";
    return 2;
  }

  try {
    const auto model = g1_model::G1Model::fromFile(argv[1]);

    for (const auto & edge : model.treeEdges()) {
      std::printf("edge %s %s\n", edge.first.c_str(), edge.second.c_str());
    }

    for (int i = 0; i < frames; ++i) {
      const double phase = 2.0 * M_PI * i / frames;
      const auto q = motionAt(model, phase);

      std::printf("frame %d\n", i);
      for (const auto & entry : model.linkPoses(q)) {
        const auto & p = entry.second.p;
        std::printf("link %s %.6f %.6f %.6f\n", entry.first.c_str(), p.x(), p.y(), p.z());
      }
      const auto com = model.centerOfMass(q);
      std::printf("com %.6f %.6f %.6f\n", com.x(), com.y(), com.z());
    }
    return 0;
  } catch (const g1_model::ModelError & e) {
    std::cerr << "g1_skeleton_dump: " << e.what() << "\n";
    return 1;
  }
}
