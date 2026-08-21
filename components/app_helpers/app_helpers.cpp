#include "components/app_helpers/app_helpers.hpp"

namespace motion_control_lab
{

Eigen::VectorXd toEigen(const std::vector<double> & values)
{
  Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
  for (std::size_t index = 0; index < values.size(); ++index) {
    result(static_cast<Eigen::Index>(index)) = values[index];
  }
  return result;
}

std::vector<double> toStdVector(const Eigen::VectorXd & values)
{
  return std::vector<double>(values.data(), values.data() + values.size());
}

CpuAffinityDebug makeCpuAffinityDebug(const CpuAffinityBinding & binding)
{
  return CpuAffinityDebug{
    binding.role, binding.enabled, binding.thread_id,
    binding.requested_cpus, binding.effective_cpus};
}

}  // namespace motion_control_lab
