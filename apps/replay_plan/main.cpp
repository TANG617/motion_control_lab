#include "adapters/replay/replay_support.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>

int main(int argc, char ** argv)
{
  namespace replay = motion_control_lab::replay;
  try {
    auto options = replay::parseReplayOptions(argc, argv, false);
    if (options.help) {
      std::cout << replay::replayHelp(argv[0], false);
      return EXIT_SUCCESS;
    }
    const auto loaded = replay::loadReplay(options);
    replay::writeReplayPlanArtifacts(options, loaded);
    std::cout << "frames=" << loaded.timeline.timeline.size()
              << " matched=" << loaded.timeline.pairing.matched_count
              << " unmatched_left=" << loaded.timeline.pairing.unmatched_left_count
              << " unmatched_right=" << loaded.timeline.pairing.unmatched_right_count
              << " output=" << options.output_dir.string() << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "mcl_replay_plan: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
