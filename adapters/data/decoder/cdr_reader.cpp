#include "adapters/data/decoder/cdr_reader.hpp"

#include <cstring>
#include <limits>

namespace motion_control_lab::data
{
namespace
{
constexpr std::uint32_t kMaximumSequenceLength = 1'000'000;
}

CdrReader::CdrReader(const std::vector<std::byte> & bytes)
  : bytes_(bytes)
{
  if (bytes_.size() < 4) {
    throw DataError(DataErrorCode::DecodeFailure, "CDR payload lacks encapsulation header");
  }
  const auto kind_high = std::to_integer<std::uint8_t>(bytes_[0]);
  const auto kind_low = std::to_integer<std::uint8_t>(bytes_[1]);
  const auto options_high = std::to_integer<std::uint8_t>(bytes_[2]);
  const auto options_low = std::to_integer<std::uint8_t>(bytes_[3]);
  if (kind_high != 0 || (kind_low != 0 && kind_low != 1) ||
      options_high != 0 || options_low != 0) {
    throw DataError(
            DataErrorCode::UnsupportedEncoding,
            "only CDR_BE and CDR_LE encapsulation with zero options is supported");
  }
  little_endian_ = kind_low == 1;
}

void CdrReader::require(std::size_t count) const
{
  if (count > bytes_.size() || position_ > bytes_.size() - count) {
    throw DataError(DataErrorCode::DecodeFailure, "CDR payload is truncated");
  }
}

void CdrReader::align(std::size_t alignment)
{
  const auto relative = position_ - 4;
  const auto padding = (alignment - (relative % alignment)) % alignment;
  require(padding);
  position_ += padding;
}

std::uint32_t CdrReader::readRawUint32()
{
  require(4);
  std::uint32_t result = 0;
  if (little_endian_) {
    for (std::size_t index = 0; index < 4; ++index) {
      result |= static_cast<std::uint32_t>(
        std::to_integer<std::uint8_t>(bytes_[position_ + index])) << (index * 8U);
    }
  } else {
    for (std::size_t index = 0; index < 4; ++index) {
      result = (result << 8U) |
        std::to_integer<std::uint8_t>(bytes_[position_ + index]);
    }
  }
  position_ += 4;
  return result;
}

std::uint64_t CdrReader::readRawUint64()
{
  require(8);
  std::uint64_t result = 0;
  if (little_endian_) {
    for (std::size_t index = 0; index < 8; ++index) {
      result |= static_cast<std::uint64_t>(
        std::to_integer<std::uint8_t>(bytes_[position_ + index])) << (index * 8U);
    }
  } else {
    for (std::size_t index = 0; index < 8; ++index) {
      result = (result << 8U) |
        std::to_integer<std::uint8_t>(bytes_[position_ + index]);
    }
  }
  position_ += 8;
  return result;
}

std::int32_t CdrReader::readInt32()
{
  align(4);
  return static_cast<std::int32_t>(readRawUint32());
}

std::uint32_t CdrReader::readUint32()
{
  align(4);
  return readRawUint32();
}

double CdrReader::readDouble()
{
  align(8);
  const auto bits = readRawUint64();
  double value = 0.0;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::string CdrReader::readString()
{
  const auto length = readUint32();
  if (length == 0 || length > kMaximumSequenceLength) {
    throw DataError(DataErrorCode::DecodeFailure, "CDR string has an invalid length");
  }
  require(length);
  const auto * start = reinterpret_cast<const char *>(bytes_.data() + position_);
  if (start[length - 1] != '\0') {
    throw DataError(DataErrorCode::DecodeFailure, "CDR string is not null terminated");
  }
  if (std::memchr(start, '\0', length - 1) != nullptr) {
    throw DataError(DataErrorCode::DecodeFailure, "CDR string contains an embedded null");
  }
  std::string result(start, start + length - 1);
  position_ += length;
  return result;
}

std::vector<std::string> CdrReader::readStringSequence()
{
  const auto length = readUint32();
  if (length > kMaximumSequenceLength) {
    throw DataError(DataErrorCode::DecodeFailure, "CDR string sequence is too large");
  }
  std::vector<std::string> result;
  result.reserve(length);
  for (std::uint32_t index = 0; index < length; ++index) {
    result.push_back(readString());
  }
  return result;
}

std::vector<double> CdrReader::readDoubleSequence()
{
  const auto length = readUint32();
  if (length > kMaximumSequenceLength) {
    throw DataError(DataErrorCode::DecodeFailure, "CDR double sequence is too large");
  }
  std::vector<double> result;
  result.reserve(length);
  for (std::uint32_t index = 0; index < length; ++index) {
    result.push_back(readDouble());
  }
  return result;
}

void CdrReader::requireFinished() const
{
  if (position_ != bytes_.size()) {
    throw DataError(DataErrorCode::DecodeFailure, "CDR payload contains trailing bytes");
  }
}

}  // namespace motion_control_lab::data
