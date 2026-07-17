#include "localization/opt/hba.hpp"
#include "localization/opt/map.hpp"
#include "localization/opt/pgo.hpp"

#include <filesystem>
#include <iostream>
#include <string>

namespace {

struct Args {
  std::string algo = "pgo";
  std::filesystem::path map_dir;
  std::filesystem::path out_dir;
};

bool has_suffix(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
      value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

Args parse_args(int argc, char** argv) {
  Args args;
  if (argc > 0) {
    const std::string exe = argv[0] == nullptr ? "" : argv[0];
    if (exe.find("hba") != std::string::npos) {
      args.algo = "hba";
    }
  }

  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i] == nullptr ? "" : argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc || argv[i + 1] == nullptr) {
        return "";
      }
      ++i;
      return argv[i];
    };
    if (key == "--map" || key == "-m") {
      args.map_dir = next();
    } else if (key == "--out" || key == "-o") {
      args.out_dir = next();
    } else if (key == "--algo") {
      args.algo = next();
    } else if (has_suffix(key, "hba")) {
      args.algo = "hba";
    } else if (has_suffix(key, "pgo")) {
      args.algo = "pgo";
    }
  }
  return args;
}

void print_json(
    const std::string& algo,
    const lingtu::localization::opt::Result& result) {
  const bool performed =
      result.ok && result.code != "skipped_no_independent_constraints";
  std::cout
      << "{\"ok\":" << (result.ok ? "true" : "false")
      << ",\"algo\":\"" << algo << "\""
      << ",\"code\":\"" << result.code << "\""
      << ",\"message\":\"" << result.message << "\""
      << ",\"patch_count\":" << result.patch_count
      << ",\"pose_count\":" << result.pose_count
      << ",\"factor_count\":" << result.factor_count
      << ",\"iterations\":" << result.iterations
      << ",\"performed\":" << (performed ? "true" : "false")
      << ",\"changed\":" << (result.changed ? "true" : "false")
      << ",\"report\":\"" << result.report_path.string() << "\""
      << "}" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  if (args.map_dir.empty()) {
    lingtu::localization::opt::Result result;
    result.code = "map_arg_missing";
    result.message = "--map is required";
    print_json(args.algo, result);
    return 2;
  }

  const auto map = lingtu::localization::opt::files(args.map_dir, args.out_dir);
  const auto result = args.algo == "hba"
      ? lingtu::localization::opt::hba(map)
      : lingtu::localization::opt::pgo(map);
  print_json(args.algo, result);
  return result.ok ? 0 : 2;
}
