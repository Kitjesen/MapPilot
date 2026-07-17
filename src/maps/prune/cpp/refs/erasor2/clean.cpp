// SPDX-License-Identifier: GPL-3.0-only
//
// Optional LingTu wrapper for the GPLv3 ERASOR2 backend.

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef _WIN32
#include <sys/wait.h>
#endif

namespace fs = std::filesystem;

namespace {

struct Options {
  fs::path map_dir;
  fs::path stage_dir;
  fs::path stage_bin;
  fs::path erasor2_bin;
  bool overwrite{false};
  bool skip_run{false};
};

std::string jsonEscape(const std::string &value) {
  std::ostringstream out;
  for (char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::string pathString(const fs::path &path) {
  return path.empty() ? std::string() : path.generic_string();
}

std::string shellQuote(const fs::path &path) {
  const std::string value = pathString(path);
  std::string out = "'";
  for (char c : value) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out += c;
    }
  }
  out += "'";
  return out;
}

fs::path exePath(fs::path path) {
#ifdef _WIN32
  if (!fs::exists(path) && path.extension().empty()) {
    path += ".exe";
  }
#endif
  return path;
}

fs::path defaultErasor2Bin(const fs::path &bin_dir) {
  const fs::path flat = exePath(bin_dir / "run_erasor2");
  if (fs::exists(flat)) {
    return flat;
  }
  return exePath(bin_dir / "erasor2_upstream" / "run_erasor2");
}

std::vector<fs::path> listPcds(const fs::path &dir) {
  std::vector<fs::path> out;
  if (!fs::is_directory(dir)) {
    return out;
  }
  for (const fs::directory_entry &entry : fs::directory_iterator(dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
      out.push_back(entry.path());
    }
  }
  return out;
}

void printHelp() {
  std::cout << "Usage: erasor2_clean --map-dir MAP_DIR [options]\n"
            << "\n"
            << "Stages a LingTu saved map and runs the optional GPLv3 ERASOR2 backend.\n"
            << "\n"
            << "Options:\n"
            << "  --map-dir PATH       LingTu map dir with map.pcd, patches/, poses.txt\n"
            << "  --stage-dir PATH     Staging directory (default: MAP_DIR/erasor2_stage)\n"
            << "  --out PATH           Alias for --stage-dir\n"
            << "  --stage-bin PATH     erasor2_stage executable path\n"
            << "  --erasor2-bin PATH   upstream run_erasor2 executable path\n"
            << "  --overwrite          Replace an existing staging directory\n"
            << "  --skip-run           Only stage data and print the upstream command\n"
            << "  --help               Show this message\n";
}

bool nextValue(int &i, int argc, char **argv, std::string &value) {
  if (i + 1 >= argc) {
    return false;
  }
  value = argv[++i];
  return true;
}

Options parseArgs(int argc, char **argv) {
  Options options;
  const fs::path self = argc > 0 ? fs::absolute(argv[0]) : fs::path();
  const fs::path bin_dir = self.empty() ? fs::current_path() : self.parent_path();
  options.stage_bin = exePath(bin_dir / "erasor2_stage");
  if (!fs::exists(options.stage_bin)) {
    options.stage_bin = exePath(bin_dir / "lingtu_erasor2_stage");
  }
  options.erasor2_bin = defaultErasor2Bin(bin_dir);

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string value;
    if (arg == "--help" || arg == "-h") {
      printHelp();
      std::exit(0);
    }
    if (arg == "--map-dir" && nextValue(i, argc, argv, value)) {
      options.map_dir = value;
    } else if ((arg == "--stage-dir" || arg == "--out") && nextValue(i, argc, argv, value)) {
      options.stage_dir = value;
    } else if (arg == "--stage-bin" && nextValue(i, argc, argv, value)) {
      options.stage_bin = exePath(value);
    } else if (arg == "--erasor2-bin" && nextValue(i, argc, argv, value)) {
      options.erasor2_bin = exePath(value);
    } else if (arg == "--overwrite") {
      options.overwrite = true;
    } else if (arg == "--skip-run") {
      options.skip_run = true;
    } else {
      std::cerr << "unknown or incomplete argument: " << arg << "\n";
      printHelp();
      std::exit(2);
    }
  }
  if (!options.map_dir.empty() && options.stage_dir.empty()) {
    options.stage_dir = fs::absolute(options.map_dir) / "erasor2_stage";
  }
  return options;
}

std::string makeStageCommand(const Options &options) {
  std::ostringstream cmd;
  cmd << shellQuote(options.stage_bin) << " --map-dir " << shellQuote(options.map_dir) << " --out "
      << shellQuote(options.stage_dir);
  if (options.overwrite) {
    cmd << " --overwrite";
  }
  cmd << " 1>&2";
  return cmd.str();
}

fs::path stageConfigPath(const fs::path &stage_dir) {
  const fs::path primary = fs::absolute(stage_dir) / "erasor2.yaml";
  if (fs::is_regular_file(primary)) {
    return primary;
  }
  const fs::path legacy = fs::absolute(stage_dir) / "lingtu_erasor2.yaml";
  if (fs::is_regular_file(legacy)) {
    return legacy;
  }
  return primary;
}

std::string makeRunCommand(const Options &options) {
  const fs::path config = stageConfigPath(options.stage_dir);
  std::ostringstream cmd;
  cmd << shellQuote(options.erasor2_bin) << " " << shellQuote(config) << " 1>&2";
  return cmd.str();
}

int normalizeSystemRc(int rc) {
  if (rc == -1) {
    return rc;
  }
#ifndef _WIN32
  if (WIFEXITED(rc)) {
    return WEXITSTATUS(rc);
  }
  if (WIFSIGNALED(rc)) {
    return 128 + WTERMSIG(rc);
  }
#endif
  return rc;
}

void printJson(const Options &options, int stage_rc, int run_rc, const std::string &reason) {
  const fs::path stage_dir = fs::absolute(options.stage_dir);
  const fs::path config = stageConfigPath(stage_dir);
  const fs::path output = stage_dir / "output";
  const auto pcds = listPcds(output);
  const bool success = stage_rc == 0 && (options.skip_run || run_rc == 0);

  std::cout << "{\n";
  std::cout << "  \"success\": " << (success ? "true" : "false") << ",\n";
  std::cout << "  \"backend\": \"erasor2\",\n";
  std::cout << "  \"license\": \"GPL-3.0-only\",\n";
  std::cout << "  \"reason_code\": \"" << jsonEscape(reason) << "\",\n";
  std::cout << "  \"map_dir\": \"" << jsonEscape(pathString(fs::absolute(options.map_dir)))
            << "\",\n";
  std::cout << "  \"stage_dir\": \"" << jsonEscape(pathString(stage_dir)) << "\",\n";
  std::cout << "  \"config_path\": \"" << jsonEscape(pathString(config)) << "\",\n";
  std::cout << "  \"output_dir\": \"" << jsonEscape(pathString(output)) << "\",\n";
  std::cout << "  \"stage_bin\": \"" << jsonEscape(pathString(options.stage_bin)) << "\",\n";
  std::cout << "  \"erasor2_bin\": \"" << jsonEscape(pathString(options.erasor2_bin)) << "\",\n";
  std::cout << "  \"stage_rc\": " << stage_rc << ",\n";
  std::cout << "  \"run_rc\": " << run_rc << ",\n";
  std::cout << "  \"skip_run\": " << (options.skip_run ? "true" : "false") << ",\n";
  std::cout << "  \"stage_command\": \"" << jsonEscape(makeStageCommand(options)) << "\",\n";
  std::cout << "  \"run_command\": \"" << jsonEscape(makeRunCommand(options)) << "\",\n";
  std::cout << "  \"output_pcds\": [";
  for (std::size_t i = 0; i < pcds.size(); ++i) {
    std::cout << (i == 0 ? "\n" : ",\n");
    std::cout << "    \"" << jsonEscape(pathString(pcds[i])) << "\"";
  }
  if (!pcds.empty()) {
    std::cout << "\n  ";
  }
  std::cout << "]\n";
  std::cout << "}\n";
}

}  // namespace

int main(int argc, char **argv) {
  const Options options = parseArgs(argc, argv);
  if (options.map_dir.empty()) {
    printJson(options, 2, -1, "missing_map_dir");
    return 2;
  }
  if (!fs::exists(options.stage_bin)) {
    printJson(options, 127, -1, "missing_stage_bin");
    return 127;
  }
  if (!options.skip_run && !fs::exists(options.erasor2_bin)) {
    printJson(options, -1, 127, "missing_erasor2_bin");
    return 127;
  }

  const std::string stage_cmd = makeStageCommand(options);
  const int stage_rc = normalizeSystemRc(std::system(stage_cmd.c_str()));
  if (stage_rc != 0) {
    printJson(options, stage_rc, -1, "stage_failed");
    return 1;
  }

  int run_rc = 0;
  if (!options.skip_run) {
    const std::string run_cmd = makeRunCommand(options);
    run_rc = normalizeSystemRc(std::system(run_cmd.c_str()));
  }
  const bool success = options.skip_run || run_rc == 0;
  printJson(options, stage_rc, run_rc, options.skip_run ? "staged" : "ran_erasor2");
  return success ? 0 : 1;
}
