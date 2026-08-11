#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

namespace motion_control_lab::data
{

enum class DiagnosticSeverity
{
  Info,
  Warning,
  Error
};

struct Diagnostic
{
  DiagnosticSeverity severity{DiagnosticSeverity::Info};
  std::string code;
  std::string message;
  std::optional<std::size_t> record_index;
};

enum class DataErrorCode
{
  Io,
  InvalidArgument,
  InvalidFormat,
  UnsupportedEncoding,
  SchemaMismatch,
  DecodeFailure,
  MissingTimestamp,
  InvalidTimestamp,
  NonMonotonicTimestamp,
  DuplicateTimestamp,
  UnmatchedSample,
  FrameMismatch
};

class DataError : public std::runtime_error
{
public:
  DataError(DataErrorCode code, std::string message)
    : std::runtime_error(std::move(message)), code_(code)
  {
  }

  DataErrorCode code() const noexcept { return code_; }

private:
  DataErrorCode code_;
};

}  // namespace motion_control_lab::data
