#include "adapters/replay/replay_support.hpp"
#include "motion_control_lab/sha256.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace data = motion_control_lab::data;
namespace replay = motion_control_lab::replay;

int main()
{
  const auto input = std::filesystem::temp_directory_path() / "mcl-manifest-input.csv";
  {
    std::ofstream output(input);
    output << "canonical-input\n";
  }
  try {
    replay::ReplayOptions options;
    options.input_path = input;
    options.input_format = replay::InputFormat::Csv;
    options.left_stream = "left";
    options.right_stream = "right";
    options.target_period_ns = 10'000'000;
    options.timestamp_projection.policy = data::TimestampPolicy::FixedPeriod;
    options.timestamp_projection.period_ns = options.target_period_ns;
    options.original_argv = {"mcl_test", "replay", "--input", input.string()};
    options.launcher = "scripts/run_csv_replay.sh";

    replay::LoadedReplay loaded;
    data::DualArmFrame value;
    loaded.timeline.timeline = data::Timeline<data::DualArmFrame>({
      {0, 0, 0, 0, data::TimestampPolicy::FixedPeriod, value}});
    loaded.timeline.pairing.left_input_count = 1;
    loaded.timeline.pairing.right_input_count = 1;
    loaded.timeline.pairing.matched_count = 1;

    replay::ReplayExecutionMetadata execution;
    execution.app = "manifest-test";
    execution.solver = "mcc";
    execution.backend = "proxqp";
    execution.rate_hz = 100.0;
    execution.resolved_config["regularization"] = "1e-4";
    const auto manifest = replay::makeReplayManifest(options, loaded, execution, "trace-hash");
    if (manifest["input"]["sha256"].asString() != motion_control_lab::sha256_file(input) ||
        manifest["invocation"]["argv"].size() != 4U ||
        manifest["invocation"]["launcher"].asString() != options.launcher ||
        manifest["resolved_config"]["regularization"].asString() != "1e-4" ||
        manifest["resolved_config"]["execution_mode"].asString() != "batch") {
      throw std::runtime_error("replay manifest omitted provenance or resolved config");
    }
    std::filesystem::remove(input);
    return EXIT_SUCCESS;
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(input, ignored);
    throw;
  }
}
