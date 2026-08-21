#include "loop.hpp"

#include <iostream>

namespace motion_control_lab::replay_plan {

void runLoop(const Options &options) {
  const auto loaded = replay::loadReplay(options);
  replay::writeReplayPlanArtifacts(options, loaded);
  std::cout << "frames=" << loaded.timeline.timeline.size()
            << " matched=" << loaded.timeline.pairing.matched_count
            << " unmatched_left="
            << loaded.timeline.pairing.unmatched_left_count
            << " unmatched_right="
            << loaded.timeline.pairing.unmatched_right_count
            << " output=" << options.output_dir.string() << '\n';
}

} // namespace motion_control_lab::replay_plan
