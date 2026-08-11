#pragma once

#include "contracts/data/data_error.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace motion_control_lab::data
{

class CdrReader
{
public:
  explicit CdrReader(const std::vector<std::byte> & bytes);

  std::int32_t readInt32();
  std::uint32_t readUint32();
  double readDouble();
  std::string readString();
  std::vector<std::string> readStringSequence();
  std::vector<double> readDoubleSequence();
  void requireFinished() const;

private:
  void align(std::size_t alignment);
  void require(std::size_t count) const;
  std::uint32_t readRawUint32();
  std::uint64_t readRawUint64();

  const std::vector<std::byte> & bytes_;
  std::size_t position_{4};
  bool little_endian_{true};
};

}  // namespace motion_control_lab::data
