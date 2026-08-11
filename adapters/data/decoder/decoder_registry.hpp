#pragma once

#include "adapters/data/source/data_source.hpp"
#include "contracts/data/typed_stream.hpp"

#include <any>
#include <memory>
#include <optional>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

namespace motion_control_lab::data
{

struct AnyDecodedSample
{
  std::any sample;
  std::vector<Diagnostic> diagnostics;
};

class RecordDecoder
{
public:
  virtual ~RecordDecoder() = default;
  virtual const std::string & id() const = 0;
  virtual std::type_index outputType() const = 0;
  virtual bool supports(const StreamDescriptor & stream) const = 0;
  virtual AnyDecodedSample decode(const EncodedRecord & record) const = 0;
};

class DecoderRegistry
{
public:
  void registerDecoder(std::shared_ptr<const RecordDecoder> decoder);

  template<typename T>
  TypedStream<T> decode(
    SourceCursor & cursor,
    std::string logical_name,
    std::optional<std::string> decoder_id = std::nullopt) const
  {
    TypedStream<T> result;
    result.logical_name = std::move(logical_name);
    const RecordDecoder * selected = nullptr;
    EncodedRecord record;
    std::size_t record_index = 0;
    while (cursor.next(record)) {
      const auto & descriptor = std::visit(
        [](const auto & value) -> const StreamDescriptor & {return value.stream;}, record);
      if (selected == nullptr) {
        selected = findDecoder(std::type_index(typeid(T)), descriptor, decoder_id);
        result.decoder_id = selected->id();
      }
      if (!selected->supports(descriptor)) {
        throw DataError(
                DataErrorCode::SchemaMismatch,
                "decoder " + selected->id() + " does not support stream " +
                descriptor.logical_name);
      }
      auto decoded = selected->decode(record);
      try {
        result.samples.push_back(std::any_cast<T>(std::move(decoded.sample)));
      } catch (const std::bad_any_cast &) {
        throw DataError(
                DataErrorCode::DecodeFailure,
                "decoder " + selected->id() + " returned the wrong sample type");
      }
      for (auto & diagnostic : decoded.diagnostics) {
        if (!diagnostic.record_index.has_value()) {
          diagnostic.record_index = record_index;
        }
        result.diagnostics.push_back(std::move(diagnostic));
      }
      ++record_index;
    }
    if (selected == nullptr && decoder_id.has_value()) {
      const auto * empty_decoder = findDecoderById(std::type_index(typeid(T)), *decoder_id);
      result.decoder_id = empty_decoder->id();
    }
    return result;
  }

private:
  const RecordDecoder * findDecoder(
    std::type_index output_type,
    const StreamDescriptor & descriptor,
    const std::optional<std::string> & decoder_id) const;
  const RecordDecoder * findDecoderById(
    std::type_index output_type,
    const std::string & decoder_id) const;

  std::vector<std::shared_ptr<const RecordDecoder>> decoders_;
};

}  // namespace motion_control_lab::data
