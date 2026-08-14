#pragma once

#include "adapters/data/source/data_source.hpp"
#include "contracts/data/typed_stream.hpp"

#include <cstddef>
#include <string>
#include <utility>

namespace motion_control_lab::data
{

template<typename Decoder>
auto decodeStream(
  SourceCursor & cursor,
  std::string logical_name,
  const Decoder & decoder) -> TypedStream<typename Decoder::Sample>
{
  using Sample = typename Decoder::Sample;

  TypedStream<Sample> result;
  result.logical_name = std::move(logical_name);
  result.decoder_id = decoder.id();

  EncodedRecord record;
  std::size_t record_index = 0;
  while (cursor.next(record)) {
    const auto & descriptor = std::visit(
      [](const auto & value) -> const StreamDescriptor & {return value.stream;}, record);
    if (!decoder.supports(descriptor)) {
      throw DataError(
              DataErrorCode::SchemaMismatch,
              "decoder " + decoder.id() + " does not support stream " +
              descriptor.logical_name);
    }

    auto decoded = decoder.decode(record);
    result.samples.push_back(std::move(decoded.sample));
    for (auto & diagnostic : decoded.diagnostics) {
      if (!diagnostic.record_index.has_value()) {
        diagnostic.record_index = record_index;
      }
      result.diagnostics.push_back(std::move(diagnostic));
    }
    ++record_index;
  }
  return result;
}

}  // namespace motion_control_lab::data
