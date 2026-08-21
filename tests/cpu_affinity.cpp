#include "components/app_helpers/cpu_affinity.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

int run()
{
  const auto domain = motion_control_lab::CpuAffinityDomain::capture();
#if MCL_ENABLE_CPU_AFFINITY
  if (!domain.enabled() || domain.allowedCpus().empty()) {
    return EXIT_FAILURE;
  }

  const auto allowed_cpu = domain.allowedCpus().front();
  const std::array<unsigned int, 1> requested{allowed_cpu};
  const auto binding = domain.bindCurrentThread("test_cpu_affinity", "valid", requested);
  if (!binding.enabled || binding.requested_cpus != binding.effective_cpus ||
      binding.effective_cpus != std::vector<unsigned int>{allowed_cpu} ||
      binding.thread_id <= 0)
  {
    return EXIT_FAILURE;
  }

  unsigned int unavailable_cpu = 0;
  while (std::find(
      domain.allowedCpus().begin(), domain.allowedCpus().end(), unavailable_cpu) !=
         domain.allowedCpus().end())
  {
    ++unavailable_cpu;
  }
  try {
    domain.bindCurrentThread(
      "test_cpu_affinity", "invalid", std::array<unsigned int, 1>{unavailable_cpu});
    return EXIT_FAILURE;
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    for (const auto * marker : {"app=test_cpu_affinity", "role=invalid", "requested=",
                               "allowed=", "effective="}) {
      if (message.find(marker) == std::string::npos) {
        return EXIT_FAILURE;
      }
    }
  }
#else
  if (domain.enabled() || !domain.allowedCpus().empty()) {
    return EXIT_FAILURE;
  }
  const auto binding = domain.bindCurrentThread(
    "test_cpu_affinity", "disabled", std::array<unsigned int, 1>{8});
  if (binding.enabled || binding.requested_cpus != std::vector<unsigned int>{8} ||
      !binding.effective_cpus.empty() || binding.thread_id != -1) {
    return EXIT_FAILURE;
  }
#endif
  return EXIT_SUCCESS;
}

}  // namespace

int main()
{
  try {
    return run();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
