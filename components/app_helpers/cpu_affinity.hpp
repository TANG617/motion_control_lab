#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace motion_control_lab
{

struct CpuAffinityBinding
{
  bool enabled{false};
  std::string app;
  std::string role;
  std::vector<unsigned int> requested_cpus;
  std::vector<unsigned int> effective_cpus;
  std::int64_t thread_id{-1};
};

class CpuAffinityDomain
{
public:
  static CpuAffinityDomain capture();

  bool enabled() const noexcept { return enabled_; }
  const std::vector<unsigned int> & allowedCpus() const noexcept { return allowed_cpus_; }

  CpuAffinityBinding describe(
    const std::string & app, const std::string & role,
    const unsigned int * cpus, std::size_t cpu_count) const;

  template<std::size_t Size>
  CpuAffinityBinding describe(
    const std::string & app, const std::string & role,
    const std::array<unsigned int, Size> & cpus) const
  {
    return describe(app, role, cpus.data(), cpus.size());
  }

  void validate(
    const std::string & app, const std::string & role,
    const unsigned int * cpus, std::size_t cpu_count) const;

  template<std::size_t Size>
  void validate(
    const std::string & app, const std::string & role,
    const std::array<unsigned int, Size> & cpus) const
  {
    validate(app, role, cpus.data(), cpus.size());
  }

  CpuAffinityBinding bindCurrentThread(
    const std::string & app, const std::string & role,
    const unsigned int * cpus, std::size_t cpu_count) const;

  template<std::size_t Size>
  CpuAffinityBinding bindCurrentThread(
    const std::string & app, const std::string & role,
    const std::array<unsigned int, Size> & cpus) const
  {
    return bindCurrentThread(app, role, cpus.data(), cpus.size());
  }

private:
  bool enabled_{false};
  std::vector<unsigned int> allowed_cpus_;
};

}  // namespace motion_control_lab
