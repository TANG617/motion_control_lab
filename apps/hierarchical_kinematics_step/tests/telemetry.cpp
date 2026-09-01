#include "../telemetry.hpp"

#include "contracts/visualization/mcl_telemetry_v1.hpp"

#include <google/protobuf/descriptor.pb.h>

#include <cstdlib>
#include <string>
#include <vector>

int main()
{
  namespace app = motion_control_lab::hierarchical_kinematics_step;
  namespace proto = app::telemetry_proto;
  namespace contract = motion_control_lab::contracts::mcl_telemetry_v1;

  app::RunClock clock;
  const auto first_clock = clock.sample();
  const auto second_clock = clock.sample();
  if (second_clock.timestamp_ns < first_clock.timestamp_ns ||
    second_clock.run_time_ns < first_clock.run_time_ns ||
    second_clock.sequence <= first_clock.sequence)
  {
    return EXIT_FAILURE;
  }

  app::WorkerTelemetryQueue bounded;
  for (std::size_t index = 0U; index < app::kTelemetryQueueCapacity; ++index) {
    if (!bounded.tryPush(app::TelemetryRecord::log(
        {1000U + index, index, index}, contract::kEventsTopic,
        motion_control::viz::LogLevel::Info, "queue-test", "sample")))
    {
      return EXIT_FAILURE;
    }
  }
  if (bounded.tryPush(app::TelemetryRecord::log(
      {9999U, 9999U, 9999U}, contract::kEventsTopic,
      motion_control::viz::LogLevel::Info, "queue-test", "overflow")) ||
    bounded.dropped() != 1U || bounded.depth() != app::kTelemetryQueueCapacity)
  {
    return EXIT_FAILURE;
  }
  std::vector<app::TelemetryRecord> discarded;
  app::drainTelemetryQueue(bounded, discarded);

  const app::EventStamp rejected_stamp{1000000100U, 100U, 10U};
  const app::EventStamp committed_stamp{1000000200U, 200U, 11U};
  const app::EventStamp emit_stamp{1000001000U, 1000U, 12U};

  proto::SolverTelemetry solver;
  *solver.mutable_context() = app::makeSampleContext(
    rejected_stamp, proto::RECOVERABLE_REJECTED,
    false, 2U, 7U, 8U, 6U);
  solver.set_solver_kind("hierarchical_ik");
  solver.set_backend("proxqp");
  auto * pass = solver.add_passes();
  pass->set_label("Primary");
  pass->set_priority(1);
  pass->set_pass(1);

  proto::JointTracking joints;
  *joints.mutable_context() = app::makeSampleContext(
    committed_stamp, proto::ACCEPTED,
    true, 2U, 7U, 9U, 7U);
  auto * joint = joints.add_joints();
  joint->set_name("left_arm_joint1");
  joint->set_execution_position_rad(0.25);

  proto::CollisionTelemetry collision;
  *collision.mutable_context() = joints.context();
  auto * pair = collision.add_pairs();
  pair->set_label("left_arm_link4--body_link4");

  app::WorkerTelemetryQueue queue;
  queue.tryPush(app::TelemetryRecord::encoded(
    committed_stamp, contract::kJointTrackingTopic,
    std::make_unique<proto::JointTracking>(joints)));
  queue.tryPush(app::TelemetryRecord::encoded(
    rejected_stamp, contract::kIkSolverTopic,
    std::make_unique<proto::SolverTelemetry>(solver)));
  queue.tryPush(app::TelemetryRecord::encoded(
    committed_stamp, contract::kCollisionTopic,
    std::make_unique<proto::CollisionTelemetry>(collision)));

  std::vector<app::TelemetryRecord> records;
  app::drainTelemetryQueue(queue, records);
  motion_control::viz::RenderBatch batch;
  batch.timestamp_ns = emit_stamp.timestamp_ns;
  app::TelemetryEncoder encoder;
  const auto statistics = encoder.append(std::move(records), emit_stamp, batch);
  if (batch.encoded_messages.size() != 3U || statistics.serialized_bytes == 0U ||
    batch.encoded_messages[0].channel != contract::kIkSolverTopic ||
    batch.encoded_messages[0].timestamp_ns != rejected_stamp.timestamp_ns ||
    batch.encoded_messages[1].channel != contract::kJointTrackingTopic ||
    batch.encoded_messages[1].timestamp_ns != committed_stamp.timestamp_ns ||
    batch.encoded_messages[1].schema_name != "mcl.telemetry.v1.JointTracking" ||
    batch.encoded_messages[1].message_encoding != "protobuf" ||
    batch.encoded_messages[1].schema_encoding != "protobuf")
  {
    return EXIT_FAILURE;
  }

  proto::SolverTelemetry parsed_solver;
  if (!parsed_solver.ParseFromArray(
      batch.encoded_messages[0].data.data(),
      static_cast<int>(batch.encoded_messages[0].data.size())) ||
    parsed_solver.context().committed() ||
    parsed_solver.context().outcome() !=
      proto::RECOVERABLE_REJECTED ||
    parsed_solver.passes(0).label() != "Primary" ||
    parsed_solver.context().timestamp().seconds() != 1 ||
    parsed_solver.context().timestamp().nanos() != 100 ||
    parsed_solver.context().emit_time().seconds() != 1 ||
    parsed_solver.context().emit_time().nanos() != 1000 ||
    parsed_solver.context().emit_time().nanos() <=
      parsed_solver.context().timestamp().nanos())
  {
    return EXIT_FAILURE;
  }

  proto::JointTracking parsed_joints;
  if (!parsed_joints.ParseFromArray(
      batch.encoded_messages[1].data.data(),
      static_cast<int>(batch.encoded_messages[1].data.size())) ||
    !parsed_joints.context().committed() ||
    parsed_joints.joints(0).name() != "left_arm_joint1")
  {
    return EXIT_FAILURE;
  }

  proto::CartesianCompliance compliance;
  *compliance.mutable_context() = joints.context();
  auto * arm = compliance.add_arms();
  arm->set_side("left");
  arm->set_reference_frame_name("base_link");
  arm->set_control_point_frame_name("left_arm_ee_link_tcp");
  arm->mutable_command_linear_acceleration_mps2()->set_x(1.25);
  std::vector<app::TelemetryRecord> compliance_records;
  compliance_records.push_back(app::TelemetryRecord::encoded(
    committed_stamp, contract::kCartesianComplianceTopic,
    std::make_unique<proto::CartesianCompliance>(compliance)));
  motion_control::viz::RenderBatch compliance_batch;
  const auto compliance_statistics = encoder.append(
    std::move(compliance_records), emit_stamp, compliance_batch);
  proto::CartesianCompliance parsed_compliance;
  if (compliance_batch.encoded_messages.size() != 1U ||
    compliance_statistics.serialized_bytes == 0U ||
    compliance_batch.encoded_messages[0].channel !=
      contract::kCartesianComplianceTopic ||
    !parsed_compliance.ParseFromArray(
      compliance_batch.encoded_messages[0].data.data(),
      static_cast<int>(compliance_batch.encoded_messages[0].data.size())) ||
    parsed_compliance.arms(0).side() != "left" ||
    parsed_compliance.arms(0).reference_frame_name() != "base_link" ||
    parsed_compliance.arms(0).control_point_frame_name() !=
      "left_arm_ee_link_tcp" ||
    parsed_compliance.arms(0).command_linear_acceleration_mps2().x() != 1.25)
  {
    return EXIT_FAILURE;
  }

  google::protobuf::FileDescriptorSet descriptor_set;
  const auto & schema = *batch.encoded_messages[0].schema_data;
  if (!descriptor_set.ParseFromArray(schema.data(), static_cast<int>(schema.size()))) {
    return EXIT_FAILURE;
  }
  bool found_mcl_schema = false;
  bool found_timestamp_schema = false;
  for (const auto & file : descriptor_set.file()) {
    found_mcl_schema = found_mcl_schema ||
      file.name().find("mcl_telemetry_v1.proto") != std::string::npos;
    found_timestamp_schema = found_timestamp_schema ||
      file.name() == "google/protobuf/timestamp.proto";
  }
  return found_mcl_schema && found_timestamp_schema ? EXIT_SUCCESS : EXIT_FAILURE;
}
