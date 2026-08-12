#include "experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/src/batch_replay.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace e03 = motion_control_lab::e03;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void writeFile(const std::filesystem::path & path, const std::string & contents)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  output << contents;
  if (!output) {
    throw std::runtime_error("failed to create test file");
  }
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ =
      std::filesystem::temp_directory_path() / ("mcl-e03-batch-test-" + std::to_string(suffix));
    std::filesystem::create_directory(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const { return path_; }

private:
  std::filesystem::path path_;
};

void testStableDirectScanAndVerification()
{
  TemporaryDirectory temporary;
  writeFile(temporary.path() / "b_action.mcap", "second");
  writeFile(temporary.path() / "a_action.mcap", "first");
  writeFile(temporary.path() / "notes.txt", "ignored");
  writeFile(temporary.path() / "nested" / "nested_action.mcap", "ignored");

  const auto actions = e03::scanMotionLibrary(temporary.path());
  require(actions.size() == 2, "scan must include only direct MCAP files");
  require(actions[0].action_id == "a_action", "actions must use stable filename ordering");
  require(actions[1].action_id == "b_action", "actions must use stable filename ordering");
  require(e03::verifyActionInput(actions[0]).unchanged, "snapshot must initially verify");
  writeFile(temporary.path() / "a_action.mcap", "other");
  require(!e03::verifyActionInput(actions[0]).unchanged, "runtime mutation must be detected");
}

void testEmptyLibrary()
{
  TemporaryDirectory temporary;
  require(e03::scanMotionLibrary(temporary.path()).empty(), "empty library must scan as empty");
}

void testUnsafeNameAndSymlinkRejected()
{
  {
    TemporaryDirectory temporary;
    writeFile(temporary.path() / ".unsafe.mcap", "unsafe");
    bool rejected = false;
    try {
      static_cast<void>(e03::scanMotionLibrary(temporary.path()));
    } catch (const std::exception &) {
      rejected = true;
    }
    require(rejected, "unsafe action IDs must be rejected");
  }
  {
    TemporaryDirectory temporary;
    writeFile(temporary.path() / "source.mcap", "source");
    std::filesystem::create_symlink(
      temporary.path() / "source.mcap", temporary.path() / "linked.mcap");
    bool rejected = false;
    try {
      static_cast<void>(e03::scanMotionLibrary(temporary.path()));
    } catch (const std::exception &) {
      rejected = true;
    }
    require(rejected, "symlinks must be rejected");
  }
}

void testBatchContinuesAfterFailure()
{
  std::vector<e03::ActionSnapshot> actions{
    {"first", "first.mcap", {}, 0, {}}, {"second", "second.mcap", {}, 0, {}}};
  std::size_t calls = 0;
  const auto records = e03::executeActionBatch(actions, [&](const e03::ActionSnapshot & action) {
    ++calls;
    if (action.action_id == "first") {
      throw std::runtime_error("first action failed");
    }
    e03::ActionExecutionRecord result;
    result.succeeded = true;
    result.frames_planned = 1;
    result.frames_attempted = 1;
    result.frames_accepted = 1;
    return result;
  });
  require(calls == 2, "batch must continue after an action exception");
  require(records.size() == 2, "batch must retain one record per action");
  require(!records[0].succeeded, "first action must remain failed");
  require(records[1].succeeded, "later action must still execute");
}

}  // namespace

int main()
{
  testStableDirectScanAndVerification();
  testEmptyLibrary();
  testUnsafeNameAndSymlinkRejected();
  testBatchContinuesAfterFailure();
  return 0;
}
