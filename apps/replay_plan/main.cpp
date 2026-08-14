#include "replay_plan.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char ** argv)
{
  namespace replay = motion_control_lab::replay_plan;
  const auto options = replay::parseOptions(argc, argv);
  if (options.help) {
    std::cout << replay::help(argv[0]);
    return EXIT_SUCCESS;
  }
  const auto loaded = replay::load(options);
  replay::writeArtifacts(options, loaded);
  std::cout
    << "frames=" << loaded.timeline.timeline.size()
    << " matched=" << loaded.timeline.pairing.matched_count
    << " unmatched_left=" << loaded.timeline.pairing.unmatched_left_count
    << " unmatched_right=" << loaded.timeline.pairing.unmatched_right_count
    << " output=" << options.output_dir.string() << '\n';
  return EXIT_SUCCESS;
}
