#include "adapters/execution/include/motion_control_lab/execution_request.hpp"
#include "options.hpp"
#include "solver.hpp"
#include <iostream>
namespace app = motion_control_lab::optimization_problem;
namespace ex = motion_control_lab::execution;
int run(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--describe-capabilities") {
    std::cout << app::capabilities() << '\n';
    return 0;
  }
  if (argc < 2 || std::string(argv[1]) == "--help") {
    std::cout << "mcl_optimization_problem --input problem.json --config "
                 "options.json --output DIR "
                 "[--dump-resolved-options]\nmcl_optimization_problem "
                 "--request FILE [--dump-resolved-options]\n";
    return 0;
  }
  ex::Request r;
  bool dump = false;
  if (ex::isRequest(argc, argv)) {
    dump = ex::requestDumpOnly(argc, argv);
    r = ex::readRequest(argv[2], "mcl_optimization_problem");
  } else {
    std::filesystem::path input, config;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--dump-resolved-options") {
        dump = true;
        continue;
      }
      if (i + 1 >= argc)
        throw std::runtime_error("missing option value");
      if (arg == "--input")
        input = argv[++i];
      else if (arg == "--config")
        config = argv[++i];
      else if (arg == "--output")
        r.output = std::filesystem::absolute(argv[++i]);
      else
        throw std::runtime_error("unsupported option: " + arg);
    }
    r.input = ex::readJson(input);
    r.document["schema_version"] = "execution_request.v1";
    r.document["app_id"] = "mcl_optimization_problem";
    r.document["execution_structure"] = "explicit_matrix";
    r.document["input"]["path"] = std::filesystem::absolute(input).string();
    r.document["input"]["sha256"] = motion_control_lab::sha256_file(input);
    r.document["input"]["format"] = "json";
    r.document["app_config"] = ex::readJson(config);
    r.document["execution"] = Json::Value(Json::objectValue);
    r.document["observation"] = Json::Value(Json::objectValue);
    r.document["tracking"] = Json::Value(Json::objectValue);
    r.document["output_dir"] = r.output.string();
    r.sha256 = "normal-no-request-file";
  }
  if (r.document["execution_structure"] != "explicit_matrix" ||
      r.document["input"]["format"] != "json")
    throw std::runtime_error("unsupported execution structure/input");
  if (!r.document["execution"].empty() || !r.document["observation"].empty())
    throw std::runtime_error(
        "explicit_matrix requires empty execution/observation");
  auto options = app::resolveOptions(r.config());
  if (dump) {
    std::cout << options << '\n';
    return 0;
  }
  ex::beginOutput(r, options, app::capabilities());
  ex::writeJson(r.output / "attempt_begin.json", r.input);
  auto result = app::solve(r.input, options);
  ex::writeJson(r.output / "result.json", result);
  return result["accepted"].asBool() ? 0 : 1;
}
int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception &e) {
    std::cerr << "mcl_optimization_problem: " << e.what() << '\n';
    return 1;
  }
}
