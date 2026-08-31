#include "localization/opt/assembly.hpp"
#include "localization/opt/constraints.hpp"
#include "localization/opt/map.hpp"
#include "localization/opt/pgo.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Args {
  std::filesystem::path map;
  std::filesystem::path out;
  std::filesystem::path constraints;
  bool auto_constraints = false;
};

std::filesystem::path normalized(const std::filesystem::path& path) {
  std::error_code error;
  const auto value = std::filesystem::weakly_canonical(path, error);
  if (error) {
    throw std::invalid_argument("failed to normalize path: " + path.string());
  }
  return value.lexically_normal();
}

bool within(const std::filesystem::path& candidate, const std::filesystem::path& parent) {
  const auto child = normalized(candidate);
  const auto directory = normalized(parent);
  auto child_it = child.begin();
  for (auto parent_it = directory.begin(); parent_it != directory.end(); ++parent_it, ++child_it) {
    if (child_it == child.end() || *child_it != *parent_it) {
      return false;
    }
  }
  return true;
}

Args parse_args(int argc, char** argv) {
  Args args;
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index] == nullptr ? "" : argv[index];
    auto next = [&]() -> std::filesystem::path {
      if (++index >= argc || argv[index] == nullptr) {
        throw std::invalid_argument("missing value for " + key);
      }
      return argv[index];
    };
    if (key == "--map") {
      args.map = next();
    } else if (key == "--out") {
      args.out = next();
    } else if (key == "--constraints") {
      args.constraints = next();
    } else if (key == "--auto-constraints") {
      args.auto_constraints = true;
    } else {
      throw std::invalid_argument("unknown argument: " + key);
    }
  }
  if (args.map.empty() || args.out.empty() ||
      (args.constraints.empty() == !args.auto_constraints)) {
    throw std::invalid_argument(
        "usage: lt_pgo --map MAP_DIR --out NEW_BUNDLE_DIR "
        "(--constraints CONSTRAINTS.txt | --auto-constraints)");
  }
  if (within(args.out, args.map)) {
    throw std::invalid_argument("--out must be outside the source map directory");
  }
  if (std::filesystem::exists(args.out)) {
    throw std::invalid_argument("--out already exists");
  }
  return args;
}

std::string escape(const std::string& value) {
  std::string out;
  for (char c : value) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out.push_back(c); break;
    }
  }
  return out;
}

void print_result(bool ok, bool performed, const std::string &code, const std::string &message,
                  std::size_t pose_count, std::size_t factor_count,
                  std::size_t sequential_count, std::size_t loop_count) {
  std::cout << "{\"ok\":" << (ok ? "true" : "false")
            << ",\"performed\":" << (performed ? "true" : "false")
            << ",\"code\":\"" << escape(code) << "\""
            << ",\"message\":\"" << escape(message) << "\""
            << ",\"pose_count\":" << pose_count << ",\"factor_count\":" << factor_count
            << ",\"sequential_count\":" << sequential_count << ",\"loop_count\":"
            << loop_count << "}" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = parse_args(argc, argv);
    const auto map = lingtu::localization::opt::files(args.map, args.out);
    std::vector<lingtu::localization::opt::GeometricConstraint> constraints;
    std::size_t sequential_count = 0;
    std::size_t loop_count = 0;
    if (args.auto_constraints) {
      const auto assembly = lingtu::localization::opt::assemble_pose_graph_constraints(map);
      if (!assembly.ready) {
        const std::size_t factor_count = assembly.sequential_count + assembly.loop_count;
        print_result(assembly.evidence_insufficient, false, assembly.code, assembly.message,
                     assembly.pose_count, factor_count,
                     assembly.sequential_count, assembly.loop_count);
        return assembly.evidence_insufficient ? 0 : 2;
      }
      auto constraints_path = args.out;
      constraints_path += ".pose_graph.constraints";
      std::string write_error;
      if (!lingtu::localization::opt::write_pose_graph_constraints_atomic(
              constraints_path, assembly.constraints, &write_error)) {
        throw std::runtime_error(write_error);
      }
      try {
        constraints = lingtu::localization::opt::read_constraints(constraints_path);
      } catch (...) {
        std::error_code cleanup_error;
        std::filesystem::remove(constraints_path, cleanup_error);
        throw;
      }
      std::error_code cleanup_error;
      std::filesystem::remove(constraints_path, cleanup_error);
      if (cleanup_error) {
        throw std::runtime_error("failed to remove private constraints file: " +
                                 cleanup_error.message());
      }
      sequential_count = assembly.sequential_count;
      loop_count = assembly.loop_count;
    } else {
      constraints = lingtu::localization::opt::read_constraints(args.constraints);
      if (constraints.empty()) {
        throw std::runtime_error("constraints file contains no graph factors");
      }
      for (const auto &constraint : constraints) {
        if (constraint.to_index == constraint.from_index + 1) {
          ++sequential_count;
        } else {
          ++loop_count;
        }
      }
    }
    const auto result = lingtu::localization::opt::pgo(
        map, constraints);
    const std::size_t input_factor_count = sequential_count + loop_count;
    if (result.ok && result.factor_count != input_factor_count) {
      print_result(false, false, "pgo_factor_count_mismatch",
                   "optimizer factor count does not match the assembled input graph",
                   result.pose_count, input_factor_count, sequential_count, loop_count);
      return 2;
    }
    const bool performed = result.ok && result.changed;
    print_result(result.ok && performed, performed, result.code, result.message, result.pose_count,
                 input_factor_count, sequential_count, loop_count);
    return result.ok && performed ? 0 : 2;
  } catch (const std::exception& exception) {
    print_result(false, false, "cli_error", exception.what(), 0, 0, 0, 0);
    return 2;
  }
}
