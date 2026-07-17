#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "localization/opt/graph.hpp"
#include "localization/opt/loop_constraints.hpp"
#include "localization/opt/map.hpp"

namespace {

struct Args {
  std::filesystem::path map_dir;
  std::filesystem::path report_path;
  lingtu::localization::opt::LoopConstraintOptions options;
};

double parse_double_arg(const std::string &key, const std::string &value) {
  errno = 0;
  char *end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (value.empty() || end == value.c_str() || end == nullptr || *end != '\0' || errno == ERANGE ||
      !std::isfinite(parsed)) {
    throw std::invalid_argument(key + " requires a finite number");
  }
  return parsed;
}

std::size_t parse_size_arg(const std::string &key, const std::string &value) {
  if (value.empty() || value.front() == '-') {
    throw std::invalid_argument(key + " requires a positive integer");
  }
  errno = 0;
  char *end = nullptr;
  const unsigned long long parsed = std::strtoull(value.c_str(), &end, 10);
  if (end == value.c_str() || end == nullptr || *end != '\0' || errno == ERANGE || parsed == 0 ||
      parsed > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(key + " requires a positive integer");
  }
  return static_cast<std::size_t>(parsed);
}

std::filesystem::path normalized_path(const std::filesystem::path &path) {
  std::error_code error;
  const auto normalized = std::filesystem::weakly_canonical(path, error);
  if (error) {
    throw std::invalid_argument("failed to normalize path " + path.string() + ": " +
                                error.message());
  }
  return normalized.lexically_normal();
}

bool path_is_within(const std::filesystem::path &candidate,
                    const std::filesystem::path &directory) {
  const auto child = normalized_path(candidate);
  const auto parent = normalized_path(directory);
  auto child_it = child.begin();
  for (auto parent_it = parent.begin(); parent_it != parent.end(); ++parent_it, ++child_it) {
    if (child_it == child.end() || *child_it != *parent_it) {
      return false;
    }
  }
  return true;
}

Args parse_args(int argc, char **argv) {
  Args args;
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index] == nullptr ? "" : argv[index];
    auto next = [&]() -> std::string {
      if (index + 1 >= argc || argv[index + 1] == nullptr) {
        throw std::invalid_argument("missing value for " + key);
      }
      ++index;
      return argv[index];
    };
    if (key == "--map" || key == "-m") {
      args.map_dir = next();
    } else if (key == "--report" || key == "-r") {
      args.report_path = next();
    } else if (key == "--xy-radius") {
      args.options.candidate_xy_radius_m = parse_double_arg(key, next());
      if (args.options.candidate_xy_radius_m <= 0.0 || args.options.candidate_xy_radius_m > 100.0) {
        throw std::invalid_argument("--xy-radius must be in (0, 100]");
      }
    } else if (key == "--z-gate") {
      args.options.candidate_max_abs_z_m = parse_double_arg(key, next());
      if (args.options.candidate_max_abs_z_m < 0.0 || args.options.candidate_max_abs_z_m > 50.0) {
        throw std::invalid_argument("--z-gate must be in [0, 50]");
      }
    } else if (key == "--min-path") {
      args.options.min_path_separation_m = parse_double_arg(key, next());
      if (args.options.min_path_separation_m < 0.0 ||
          args.options.min_path_separation_m > 100000.0) {
        throw std::invalid_argument("--min-path must be in [0, 100000]");
      }
    } else if (key == "--max-candidates") {
      args.options.max_total_candidates = parse_size_arg(key, next());
      if (args.options.max_total_candidates > 4096) {
        throw std::invalid_argument("--max-candidates must be in [1, 4096]");
      }
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }
  return args;
}

std::string escape(const std::string &value) {
  std::string output;
  output.reserve(value.size());
  for (char character : value) {
    if (character == '\\' || character == '"') {
      output.push_back('\\');
    }
    output.push_back(character);
  }
  return output;
}

}  // namespace

int main(int argc, char **argv) {
  Args args;
  try {
    args = parse_args(argc, argv);
    if (args.map_dir.empty() || args.report_path.empty()) {
      throw std::invalid_argument("usage: lt_loop_verify --map MAP_DIR --report REPORT.json");
    }
    if (args.report_path.extension() != ".json") {
      throw std::invalid_argument("--report must use a .json filename");
    }
    if (path_is_within(args.report_path, args.map_dir)) {
      throw std::invalid_argument(
          "--report must be outside the source map directory in shadow mode");
    }
    if (std::filesystem::exists(args.report_path)) {
      throw std::invalid_argument("--report already exists; choose a new audit path");
    }
  } catch (const std::exception &exception) {
    std::cerr << "lt_loop_verify: " << exception.what() << "\n";
    return 2;
  }
  const auto map = lingtu::localization::opt::files(args.map_dir);
  lingtu::localization::opt::LoopConstraintResult result;
  try {
    const auto keyframes = lingtu::localization::opt::read_poses(map.poses_txt);
    result = lingtu::localization::opt::generate_loop_constraints(map, keyframes, args.options);
  } catch (const std::exception &exception) {
    result.ok = false;
    result.code = "loop_cli_failed";
    result.message = exception.what();
  }

  std::string report_error;
  const bool report_written = lingtu::localization::opt::write_loop_constraints_report(
      args.report_path, result, &report_error);
  std::cout << "{\"ok\":" << (result.ok && report_written ? "true" : "false")
            << ",\"mode\":\"shadow\""
            << ",\"code\":\"" << escape(result.code) << "\""
            << ",\"candidate_count\":" << result.report.candidate_count
            << ",\"verified_count\":" << result.report.geometrically_verified_count
            << ",\"accepted_constraint_count\":" << result.report.accepted_constraint_count
            << ",\"consensus_support_count\":" << result.report.consensus_support_count
            << ",\"report\":\"" << escape(args.report_path.string()) << "\""
            << ",\"report_error\":\"" << escape(report_error) << "\"}" << std::endl;
  return result.ok && report_written ? 0 : 2;
}
