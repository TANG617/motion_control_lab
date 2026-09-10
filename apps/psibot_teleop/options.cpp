#include "options.hpp"
#include "contracts/input/input_contract.hpp"
#include <getopt.h>
#include <stdexcept>

namespace motion_control_lab::psibot_teleop {
std::string helpText() {
  return R"(Usage: mcl_psibot_teleop [options]
Psi SDK keyboard client. Starts in observation mode; no automatic power or motion.
  --address HOST:PORT     Psi endpoint (127.0.0.1:6060)
  --auth USER:PASS        SDK credentials (admin:admin)
  --mode single|wbc      Initial client mode (single)
  --side left|right      Selected arm (left)
  --ui tui|none          TUI or plain observation only (tui)
  --step-m VALUE         Translation step in metres (0.005)
  --rotation-step-deg N  Local TCP rotation step (5)
  --joint-step-deg N     JointPosition jog step in degrees (1)
  --velocity N           Relative velocity (0.1)
  --acceleration N       Relative acceleration (0.5)
  --jerk N               Relative jerk (1.0)
  --ui-rate HZ           UI update rate (30)
  --feedback-rate HZ     Desired feedback polling rate (20)
  --send-rate HZ         Maximum changed-target send rate (50)
  --connect-timeout SEC  Initial connection wait (5)
  --duration SEC        Exit after this duration; 0 is unlimited
  --log-dir PATH        New directory for sdk.log and session.json
  --help                Print this help without connecting

Space start/pause sending; arrows left/right select arm; w/s a/d q/e translate.
n select TCP rotation axis; u/i rotate; arrows up/down adjust step; m enter step.
JointPosition: left/right select joint; w/s +/- angle; up/down angular step x2 /2.
r reset selected target from actual; c choose Joint / Cartesian / WBC; x/Esc quit.
1..5/F1..F5/Tab pages; h/? help; PgUp/PgDn/Home/End scroll.
Paused sending does not brake the robot. Accepted RPC does not mean motion complete.
)";
}
Options parseOptions(int argc, char ** argv) {
  Options out;
  enum { Address=256, Auth, ModeArg, Side, Ui, Step, Rotation, Velocity, Acceleration,
         Jerk, UiRate, FeedbackRate, SendRate, ConnectTimeout, Duration, LogDir, JointStep };
  const option specs[] = {
    {"help", no_argument, nullptr, 'h'}, {"address", required_argument, nullptr, Address},
    {"auth", required_argument, nullptr, Auth}, {"mode", required_argument, nullptr, ModeArg},
    {"side", required_argument, nullptr, Side}, {"ui", required_argument, nullptr, Ui},
    {"step-m", required_argument, nullptr, Step}, {"rotation-step-deg", required_argument, nullptr, Rotation},
    {"joint-step-deg", required_argument, nullptr, JointStep},
    {"velocity", required_argument, nullptr, Velocity}, {"acceleration", required_argument, nullptr, Acceleration},
    {"jerk", required_argument, nullptr, Jerk}, {"ui-rate", required_argument, nullptr, UiRate},
    {"feedback-rate", required_argument, nullptr, FeedbackRate}, {"send-rate", required_argument, nullptr, SendRate},
    {"connect-timeout", required_argument, nullptr, ConnectTimeout}, {"duration", required_argument, nullptr, Duration},
    {"log-dir", required_argument, nullptr, LogDir}, {nullptr, 0, nullptr, 0}};
  optind = 0;
  for (int key; (key = getopt_long(argc, argv, "h", specs, nullptr)) != -1;) {
    switch (key) {
      case 'h': out.help = true; break;
      case Address: out.address = optarg; break;
      case Auth: out.auth = optarg; break;
      case ModeArg:
        if (std::string(optarg) == "single") out.mode = Mode::Single;
        else if (std::string(optarg) == "wbc") out.mode = Mode::Wbc;
        else throw std::invalid_argument("--mode expects single or wbc");
        break;
      case Side: out.teleop.side = armSideName(parseArmSide(optarg)); break;
      case Ui:
        if (std::string(optarg) == "tui") out.tui = true;
        else if (std::string(optarg) == "none") out.tui = false;
        else throw std::invalid_argument("--ui expects tui or none");
        break;
      case Step: out.teleop.step_m = std::stod(optarg); break;
      case Rotation: out.teleop.rotation_step_deg = std::stod(optarg); break;
      case JointStep: out.joint_step_deg = std::stod(optarg); break;
      case Velocity: out.velocity = std::stod(optarg); break;
      case Acceleration: out.acceleration = std::stod(optarg); break;
      case Jerk: out.jerk = std::stod(optarg); break;
      case UiRate: out.ui_hz = std::stod(optarg); break;
      case FeedbackRate: out.feedback_hz = std::stod(optarg); break;
      case SendRate: out.send_hz = std::stod(optarg); break;
      case ConnectTimeout: out.connect_timeout_s = std::stod(optarg); break;
      case Duration: out.duration_s = std::stod(optarg); break;
      case LogDir: out.log_dir = optarg; break;
      default: throw std::invalid_argument("Invalid command line; see --help");
    }
  }
  if (optind != argc) throw std::invalid_argument("Unexpected positional argument; see --help");
  return out;
}
}  // namespace motion_control_lab::psibot_teleop
