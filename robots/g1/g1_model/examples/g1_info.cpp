// Prints a summary of a G1 URDF: joints, limits, foot poses and centre of mass.
//
//   ros2 run g1_model g1_info <path-to-g1_XXdof.urdf>

#include <cstdio>
#include <iostream>

#include "g1_model/g1_model.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: g1_info <urdf-file>\n";
    return 2;
  }

  try {
    const auto model = g1_model::G1Model::fromFile(argv[1]);
    const auto q = model.zeroConfiguration();

    std::printf(
      "variant   : %s (%zu DOF)\nroot link : %s\ntotal mass: %.3f kg\n",
      g1_model::to_string(model.variant()).c_str(), model.dof(), model.rootLink().c_str(),
      model.totalMass());

    std::printf("\n%-28s %8s %8s %8s %8s\n", "joint", "lower", "upper", "vel", "effort");
    for (std::size_t i = 0; i < model.dof(); ++i) {
      const auto & l = model.limits()[i];
      std::printf(
        "%-28s %8.3f %8.3f %8.1f %8.1f\n", model.jointNames()[i].c_str(), l.lower, l.upper,
        l.velocity, l.effort);
    }

    const auto com = model.centerOfMass(q);
    std::printf("\nat the zero pose, in the %s frame:\n", model.rootLink().c_str());
    for (const auto & foot : {"left_ankle_roll_link", "right_ankle_roll_link"}) {
      const auto p = model.linkPose(q, foot).p;
      std::printf("  %-22s [%7.3f %7.3f %7.3f] m\n", foot, p.x(), p.y(), p.z());
    }
    std::printf("  %-22s [%7.3f %7.3f %7.3f] m\n", "center of mass", com.x(), com.y(), com.z());
    return 0;
  } catch (const g1_model::ModelError & e) {
    std::cerr << "g1_info: " << e.what() << "\n";
    return 1;
  }
}
