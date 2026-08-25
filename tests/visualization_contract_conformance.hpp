#pragma once

#include "contracts/visualization/channel_spec.hpp"

#include <motion_control_viz/render_batch.hpp>

#include <algorithm>
#include <array>

namespace motion_control_lab::tests
{

template<std::size_t Size>
bool requiredChannelsPresent(
  const motion_control::viz::RenderBatch & batch,
  const std::array<contracts::ChannelSpec, Size> & channels)
{
  return std::all_of(
    channels.begin(), channels.end(),
    [&batch](const contracts::ChannelSpec & channel) {
      if (!channel.required) {
        return true;
      }
      switch (channel.kind) {
        case contracts::ChannelKind::PoseInFrame:
          return std::any_of(
            batch.poses.begin(), batch.poses.end(),
            [&channel](const auto & pose) { return pose.channel == channel.topic; });
        case contracts::ChannelKind::JointStates:
          return std::any_of(
            batch.joint_states.begin(), batch.joint_states.end(),
            [&channel](const auto & joints) { return joints.channel == channel.topic; });
        case contracts::ChannelKind::SceneUpdate:
          return std::any_of(
            batch.line_strips.begin(), batch.line_strips.end(),
            [&channel](const auto & line) { return line.channel == channel.topic; });
        case contracts::ChannelKind::Log:
          return std::any_of(
            batch.logs.begin(), batch.logs.end(),
            [&channel](const auto & log) { return log.channel == channel.topic; });
        case contracts::ChannelKind::EncodedMessage:
          return std::any_of(
            batch.encoded_messages.begin(), batch.encoded_messages.end(),
            [&channel](const auto & sample) {
              return sample.channel == channel.topic && sample.schema_name == channel.schema;
            });
      }
      return false;
    });
}

}  // namespace motion_control_lab::tests
