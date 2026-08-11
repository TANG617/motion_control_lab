#include "adapters/data/decoder/decoder_registry.hpp"

#include <algorithm>

namespace motion_control_lab::data
{

void DecoderRegistry::registerDecoder(std::shared_ptr<const RecordDecoder> decoder)
{
  if (!decoder || decoder->id().empty()) {
    throw DataError(DataErrorCode::InvalidArgument, "decoder id must be non-empty");
  }
  const auto duplicate = std::find_if(
    decoders_.begin(), decoders_.end(),
    [&](const auto & current) {return current->id() == decoder->id();});
  if (duplicate != decoders_.end()) {
    throw DataError(DataErrorCode::InvalidArgument, "duplicate decoder id: " + decoder->id());
  }
  decoders_.push_back(std::move(decoder));
}

const RecordDecoder * DecoderRegistry::findDecoder(
  std::type_index output_type,
  const StreamDescriptor & descriptor,
  const std::optional<std::string> & decoder_id) const
{
  if (decoder_id.has_value()) {
    const auto * decoder = findDecoderById(output_type, *decoder_id);
    if (!decoder->supports(descriptor)) {
      throw DataError(
              DataErrorCode::SchemaMismatch,
              "decoder " + *decoder_id + " does not support stream " +
              descriptor.logical_name);
    }
    return decoder;
  }
  for (const auto & decoder : decoders_) {
    if (decoder->outputType() == output_type && decoder->supports(descriptor)) {
      return decoder.get();
    }
  }
  throw DataError(
          DataErrorCode::UnsupportedEncoding,
          "no registered decoder supports stream " + descriptor.logical_name);
}

const RecordDecoder * DecoderRegistry::findDecoderById(
  std::type_index output_type,
  const std::string & decoder_id) const
{
  for (const auto & decoder : decoders_) {
    if (decoder->id() == decoder_id && decoder->outputType() == output_type) {
      return decoder.get();
    }
  }
  throw DataError(
          DataErrorCode::InvalidArgument,
          "decoder is not registered for requested sample type: " + decoder_id);
}

}  // namespace motion_control_lab::data
