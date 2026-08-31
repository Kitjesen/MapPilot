#include "localization/opt/assembly.hpp"
#include "localization/opt/cloud.hpp"
#include "localization/opt/constraints.hpp"
#include "localization/opt/loop_constraints.hpp"
#include "localization/opt/graph.hpp"
#include "localization/opt/map.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace opt = lingtu::localization::opt;

namespace {

std::string shell_quote(const std::filesystem::path &path) {
#ifdef _WIN32
  std::string value = path.string();
  std::string escaped;
  for (char c : value) {
    escaped += c == '"' ? "\"\"" : std::string(1, c);
  }
  return "\"" + escaped + "\"";
#else
  std::string value = path.string();
  std::string escaped = "'";
  for (char c : value) {
    escaped += c == '\'' ? "'\\''" : std::string(1, c);
  }
  return escaped + "'";
#endif
}

std::pair<int, std::string> run_capture(const std::string &command) {
#ifdef _WIN32
  FILE *pipe = _popen(command.c_str(), "r");
#else
  FILE *pipe = popen(command.c_str(), "r");
#endif
  if (pipe == nullptr) {
    return {-1, {}};
  }
  std::string output;
  char buffer[4096];
  while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }
#ifdef _WIN32
  const int status = _pclose(pipe);
#else
  const int status = pclose(pipe);
#endif
  return {status, output};
}

std::string auto_pgo_command(const std::filesystem::path &executable,
                             const std::filesystem::path &map,
                             const std::filesystem::path &output) {
#ifdef _WIN32
  return "call " + shell_quote(executable) + " --map " + shell_quote(map) + " --out " +
         shell_quote(output) + " --auto-constraints";
#else
  return shell_quote(executable) + " --map " + shell_quote(map) + " --out " +
         shell_quote(output) + " --auto-constraints";
#endif
}

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::vector<opt::Point> scene_pattern(double body_x, float amplitude, float phase) {
  std::vector<opt::Point> points;
  for (int a = -12; a <= 12; ++a) {
    for (int b = -12; b <= 12; ++b) {
      const float u = static_cast<float>(a) * 0.20F;
      const float v = static_cast<float>(b) * 0.20F;
      const float noise = amplitude * std::sin(0.7F * u + 0.3F * v + phase);
      points.push_back({u - static_cast<float>(body_x), v, noise, 1.0F});
      points.push_back({u - static_cast<float>(body_x), -2.2F + noise, v, 1.0F});
      points.push_back({2.4F + noise - static_cast<float>(body_x), u, v, 1.0F});
    }
  }
  return points;
}

std::vector<opt::Point> scene(double body_x, bool perturb) {
  return scene_pattern(body_x, perturb ? 0.006F : 0.0F, 0.0F);
}

void write_cloud(const std::filesystem::path &path, const std::vector<opt::Point> &points) {
  std::ofstream out(path, std::ios::binary);
  out << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
         "COUNT 1 1 1 1\nWIDTH "
      << points.size() << "\nHEIGHT 1\nPOINTS " << points.size() << "\nDATA binary\n";
  out.write(reinterpret_cast<const char *>(points.data()),
            static_cast<std::streamsize>(points.size() * sizeof(opt::Point)));
}

void test_synthetic_sequential_constraint(const std::filesystem::path &root,
                                          const std::filesystem::path &pgo_executable) {
  const auto map_dir = root / "map";
  std::filesystem::create_directories(map_dir / "patches");
  std::ofstream(map_dir / "poses.txt") << "0.pcd 0 0 0 1 0 0 0\n"
                                         "1.pcd 1 0 0 1 0 0 0\n";
  write_cloud(map_dir / "patches" / "0.pcd", scene(0.0, false));
  write_cloud(map_dir / "patches" / "1.pcd", scene(1.0, true));
  write_cloud(map_dir / "map.pcd", scene(0.0, false));
  const auto keyframes = opt::read_poses(map_dir / "poses.txt");
  const auto result = opt::generate_sequential_constraint(opt::files(map_dir), keyframes, 0);
  require(result.ok, "synthetic adjacent registration failed: " + result.code + " / " + result.message);
  require(result.constraint.from_index == 0 && result.constraint.to_index == 1,
          "sequential edge indices changed");
  require(std::abs(result.constraint.pose_from_to.x - 1.0) < 0.03,
          "sequential measurement did not recover translation");
  require(opt::valid_information_upper(result.constraint.information_upper),
          "sequential measurement did not carry measured information");

  const auto constraints_path = map_dir / "pose_graph.constraints";
  std::string write_error;
  require(opt::write_pose_graph_constraints_atomic(constraints_path, {result.constraint}, &write_error),
          "atomic constraint writer failed: " + write_error);
  const auto roundtrip = opt::read_constraints(constraints_path);
  require(roundtrip.size() == 1 && roundtrip.front().from_index == 0 &&
              roundtrip.front().to_index == 1,
          "atomic constraint writer did not round-trip");
  std::filesystem::remove(constraints_path);

  const auto assembly = opt::assemble_pose_graph_constraints(opt::files(map_dir));
  require(!assembly.ready && assembly.evidence_insufficient &&
              assembly.code == "no_verified_loops",
          "automatic assembly accepted a graph without a verified loop");
  require(!std::filesystem::exists(constraints_path),
          "not-ready assembly published a partial constraints file");

  opt::GeometricConstraint loop_a = result.constraint;
  loop_a.from_index = 0;
  loop_a.to_index = 9;
  opt::GeometricConstraint loop_b = result.constraint;
  loop_b.from_index = 0;
  loop_b.to_index = 8;
  const auto merged = opt::detail::merge_pose_graph_constraints(
      {result.constraint}, {loop_a, loop_b});
  require(merged.size() == 3 && merged[0].to_index == 1 && merged[1].to_index == 8 &&
              merged[2].to_index == 9,
          "constraint ordering is not sequential-chain-then-sorted-loops");

  if (!pgo_executable.empty()) {
    const auto output = root / "auto-output";
    const auto [exit_status, stdout_json] =
        run_capture(auto_pgo_command(pgo_executable, map_dir, output));
    require(exit_status == 0, "auto-constraints skip did not exit zero");
    require(stdout_json.find("\"ok\":true") != std::string::npos &&
                stdout_json.find("\"performed\":false") != std::string::npos &&
                stdout_json.find("\"code\":\"no_verified_loops\"") != std::string::npos &&
                stdout_json.find("\"factor_count\":1") != std::string::npos &&
                stdout_json.find("\"sequential_count\":1") != std::string::npos &&
                stdout_json.find("\"loop_count\":0") != std::string::npos,
            "auto-constraints skip stdout contract changed: " + stdout_json);
    require(!std::filesystem::exists(output), "auto skip created an output directory");
    require(!std::filesystem::exists(map_dir / "pose_graph.constraints"),
            "auto skip polluted the source map");
    auto private_path = output;
    private_path += ".pose_graph.constraints";
    require(!std::filesystem::exists(private_path), "auto skip left a private constraints file");
  }

  write_cloud(map_dir / "patches" / "1.pcd", {{0.0F, 0.0F, 0.0F, 1.0F}});
  const auto incomplete = opt::assemble_pose_graph_constraints(opt::files(map_dir));
  require(!incomplete.ready && incomplete.evidence_insufficient &&
              incomplete.code == "sequential_chain_incomplete",
          "incomplete adjacent chain did not fail with its exact code");
  require(!std::filesystem::exists(constraints_path),
          "incomplete adjacent chain published a partial constraints file");

  std::ofstream(map_dir / "poses.txt", std::ios::binary | std::ios::trunc)
      << "this is not a pose row\n";
  const auto malformed = opt::assemble_pose_graph_constraints(opt::files(map_dir));
  require(!malformed.ready && !malformed.evidence_insufficient &&
              malformed.code == "poses_read_failed",
          "malformed poses were treated as insufficient geometric evidence");
  if (!pgo_executable.empty()) {
    const auto output = root / "malformed-output";
    const auto [exit_status, stdout_json] =
        run_capture(auto_pgo_command(pgo_executable, map_dir, output));
    require(exit_status != 0, "malformed poses returned a successful auto-PGO exit status");
    require(stdout_json.find("\"ok\":false") != std::string::npos &&
                stdout_json.find("\"performed\":false") != std::string::npos &&
                stdout_json.find("\"code\":\"poses_read_failed\"") != std::string::npos,
            "malformed poses did not return the fatal CLI contract: " + stdout_json);
    require(!std::filesystem::exists(output), "fatal auto-PGO input created an output directory");
  }
}

void test_closed_trajectory_assembly(const std::filesystem::path &root,
                                     const std::filesystem::path &pgo_executable) {
  const auto map_dir = root / "closed-map";
  std::filesystem::create_directories(map_dir / "patches");
  std::vector<double> positions;
  for (int x = 0; x <= 12; ++x) {
    positions.push_back(static_cast<double>(x));
  }
  for (int x = 11; x >= 0; --x) {
    positions.push_back(static_cast<double>(x));
  }
  std::ofstream poses(map_dir / "poses.txt");
  for (std::size_t index = 0; index < positions.size(); ++index) {
    const std::string name = std::to_string(index) + ".pcd";
    const double estimated_x = positions[index] + 0.005 * static_cast<double>(index);
    poses << name << ' ' << estimated_x << " 0 0 1 0 0 0\n";
    write_cloud(map_dir / "patches" / name,
                scene_pattern(positions[index], 0.006F, static_cast<float>(index) * 0.37F));
  }
  poses.close();
  write_cloud(map_dir / "map.pcd", scene(0.0, false));

  const auto assembly = opt::assemble_pose_graph_constraints(opt::files(map_dir));
  std::string loop_diagnostics;
  if (!assembly.ready) {
    const auto loops = opt::generate_loop_constraints(
        opt::files(map_dir), opt::read_poses(map_dir / "poses.txt"));
    for (const auto &candidate : loops.report.candidates) {
      loop_diagnostics += " [" + std::to_string(candidate.from_index) + "->" +
                          std::to_string(candidate.to_index) + ":" + candidate.reason + "]";
    }
  }
  require(assembly.ready,
          "closed trajectory did not assemble: " + assembly.code + " / " + assembly.message +
              loop_diagnostics);
  require(assembly.sequential_count == positions.size() - 1,
          "closed trajectory did not contain its full sequential chain");
  require(assembly.loop_count >= 1, "closed trajectory did not produce a verified loop");
  require(assembly.constraints.size() == assembly.sequential_count + assembly.loop_count,
          "assembled factor counts do not match the graph");

  const auto path = map_dir / "closed.constraints";
  std::string write_error;
  require(opt::write_pose_graph_constraints_atomic(path, assembly.constraints, &write_error),
          "closed graph writer failed: " + write_error);
  const auto parsed = opt::read_constraints(path);
  require(parsed.size() == assembly.constraints.size(),
          "strict parser did not round-trip the assembled closed graph");

  if (!pgo_executable.empty()) {
    const auto fatal_output = root / "closed-missing-manifest";
    const auto [fatal_status, fatal_json] =
        run_capture(auto_pgo_command(pgo_executable, map_dir, fatal_output));
    require(fatal_status != 0, "auto-PGO accepted a map without a patch bundle manifest");
    require(fatal_json.find("\"code\":\"patch_bundle_manifest_missing\"") !=
                std::string::npos &&
                fatal_json.find("\"factor_count\":" +
                                std::to_string(assembly.constraints.size())) != std::string::npos,
            "auto-PGO did not preserve the fatal code and input factor count: " + fatal_json);

    std::ofstream(map_dir / "patch_bundle.manifest")
        << "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\nfirst_sequence 0\n"
        << "last_sequence " << (positions.size() - 1) << "\npatch_count " << positions.size()
        << "\n";
    const auto output = root / "closed-auto-output";
    const auto [exit_status, stdout_json] =
        run_capture(auto_pgo_command(pgo_executable, map_dir, output));
    require(exit_status == 0, "real auto-PGO success path returned a failure: " + stdout_json);
    require(stdout_json.find("\"ok\":true") != std::string::npos &&
                stdout_json.find("\"performed\":true") != std::string::npos &&
                stdout_json.find("\"code\":\"optimized\"") != std::string::npos &&
                stdout_json.find("\"pose_count\":" + std::to_string(positions.size())) !=
                    std::string::npos &&
                stdout_json.find("\"factor_count\":" +
                                 std::to_string(assembly.constraints.size())) != std::string::npos &&
                stdout_json.find("\"sequential_count\":" +
                                 std::to_string(assembly.sequential_count)) != std::string::npos &&
                stdout_json.find("\"loop_count\":" + std::to_string(assembly.loop_count)) !=
                    std::string::npos,
            "real auto-PGO stdout contract changed: " + stdout_json);
    require(std::filesystem::is_regular_file(output / "map_optimization.json"),
            "real auto-PGO did not publish its optimization report");
    require(!std::filesystem::exists(output / "pose_graph.constraints"),
            "real auto-PGO published private constraints");
    auto private_path = output;
    private_path += ".pose_graph.constraints";
    require(!std::filesystem::exists(private_path),
            "real auto-PGO left its private constraints file");
  }
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const auto root = std::filesystem::temp_directory_path() / "lingtu-constraint-assembly-test";
    std::error_code error;
    std::filesystem::remove_all(root, error);
    std::filesystem::create_directories(root);
    test_synthetic_sequential_constraint(root, argc > 1 ? argv[1] : "");
    test_closed_trajectory_assembly(root, argc > 1 ? argv[1] : "");
    std::filesystem::remove_all(root, error);
    std::cout << "constraint_assembly_test: passed\n";
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "constraint_assembly_test: " << exception.what() << '\n';
    return 1;
  }
}
