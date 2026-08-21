#pragma once

#include "components/app_helpers/cpu_affinity.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"

#include <Eigen/Core>

#include <vector>

namespace motion_control_lab
{

Eigen::VectorXd toEigen(const std::vector<double> & values);
std::vector<double> toStdVector(const Eigen::VectorXd & values);
CpuAffinityDebug makeCpuAffinityDebug(const CpuAffinityBinding & binding);
}  // namespace motion_control_lab
