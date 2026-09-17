#pragma once
#include "loop.hpp"
namespace motion_control_lab::joint_path_planning {
class Visualization {
public:
  motion_control::viz::RenderBatch render(const Snapshot &, const Options &,
                                          const mcc::RobotModel &,
                                          std::uint64_t);

private:
  bool initialized_{false};
  std::uint64_t refresh_{0};
  std::shared_ptr<const Attempt> attempt_;
  std::vector<std::string> path_entities_;
};
} // namespace motion_control_lab::joint_path_planning
