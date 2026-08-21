#include "components/app_helpers/cpu_affinity.hpp"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <system_error>

#if MCL_ENABLE_CPU_AFFINITY
#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace motion_control_lab
{
namespace
{

std::string cpuList(const std::vector<unsigned int> & cpus)
{
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0; index < cpus.size(); ++index) {
    if (index != 0) output << ',';
    output << cpus[index];
  }
  output << ']';
  return output.str();
}

std::vector<unsigned int> normalizedCpuList(const unsigned int * cpus, std::size_t cpu_count)
{
  std::vector<unsigned int> result(cpus, cpus + cpu_count);
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  if (result.empty()) throw std::runtime_error("CPU affinity request must not be empty");
  return result;
}

std::string bindingContext(
  const std::string & app, const std::string & role,
  const std::vector<unsigned int> & requested,
  const std::vector<unsigned int> & allowed,
  const std::vector<unsigned int> & effective)
{
  return "app=" + app + " role=" + role + " requested=" + cpuList(requested) +
    " allowed=" + cpuList(allowed) + " effective=" + cpuList(effective);
}

#if MCL_ENABLE_CPU_AFFINITY
std::vector<unsigned int> currentThreadAffinity()
{
  cpu_set_t mask;
  CPU_ZERO(&mask);
  const int error = pthread_getaffinity_np(pthread_self(), sizeof(mask), &mask);
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), "pthread_getaffinity_np failed");
  }
  std::vector<unsigned int> cpus;
  for (unsigned int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (CPU_ISSET(cpu, &mask)) cpus.push_back(cpu);
  }
  return cpus;
}

std::int64_t currentThreadId()
{
  return static_cast<std::int64_t>(::syscall(SYS_gettid));
}
#endif

}  // namespace

CpuAffinityDomain CpuAffinityDomain::capture()
{
  CpuAffinityDomain result;
#if MCL_ENABLE_CPU_AFFINITY
  result.enabled_ = true;
  result.allowed_cpus_ = currentThreadAffinity();
#endif
  return result;
}

CpuAffinityBinding CpuAffinityDomain::describe(
  const std::string & app, const std::string & role,
  const unsigned int * cpus, std::size_t cpu_count) const
{
  CpuAffinityBinding binding;
  binding.enabled = enabled_;
  binding.app = app;
  binding.role = role;
  binding.requested_cpus = normalizedCpuList(cpus, cpu_count);
  return binding;
}

void CpuAffinityDomain::validate(
  const std::string & app, const std::string & role,
  const unsigned int * cpus, std::size_t cpu_count) const
{
  const auto requested_cpus = normalizedCpuList(cpus, cpu_count);
#if MCL_ENABLE_CPU_AFFINITY
  const auto effective = currentThreadAffinity();
  if (!std::includes(
      allowed_cpus_.begin(), allowed_cpus_.end(), requested_cpus.begin(), requested_cpus.end())) {
    throw std::runtime_error(
      "CPU affinity request is outside the launch allowed set: " +
      bindingContext(app, role, requested_cpus, allowed_cpus_, effective));
  }
#else
  static_cast<void>(app);
  static_cast<void>(role);
#endif
}

CpuAffinityBinding CpuAffinityDomain::bindCurrentThread(
  const std::string & app, const std::string & role,
  const unsigned int * cpus, std::size_t cpu_count) const
{
  CpuAffinityBinding binding = describe(app, role, cpus, cpu_count);
  validate(app, role, binding.requested_cpus.data(), binding.requested_cpus.size());
#if MCL_ENABLE_CPU_AFFINITY
  const auto effective_before = currentThreadAffinity();
  cpu_set_t requested_mask;
  CPU_ZERO(&requested_mask);
  for (const auto cpu : binding.requested_cpus) {
    if (cpu >= CPU_SETSIZE) {
      throw std::runtime_error(
        "CPU affinity request exceeds CPU_SETSIZE: " +
        bindingContext(app, role, binding.requested_cpus, allowed_cpus_, effective_before));
    }
    CPU_SET(cpu, &requested_mask);
  }
  const int error = pthread_setaffinity_np(pthread_self(), sizeof(requested_mask), &requested_mask);
  if (error != 0) {
    throw std::runtime_error(
      "pthread_setaffinity_np failed (" + std::to_string(error) + "): " +
      bindingContext(app, role, binding.requested_cpus, allowed_cpus_, effective_before));
  }
  binding.effective_cpus = currentThreadAffinity();
  if (binding.effective_cpus != binding.requested_cpus) {
    throw std::runtime_error(
      "CPU affinity verification failed: " +
      bindingContext(app, role, binding.requested_cpus, allowed_cpus_, binding.effective_cpus));
  }
  binding.thread_id = currentThreadId();
  std::ostringstream diagnostic;
  diagnostic << "cpu_affinity app=" << app << " role=" << role << " tid=" << binding.thread_id
             << " effective=" << cpuList(binding.effective_cpus) << '\n';
  static std::mutex diagnostic_mutex;
  const std::lock_guard<std::mutex> lock(diagnostic_mutex);
  std::clog << diagnostic.str();
#endif
  return binding;
}

}  // namespace motion_control_lab
