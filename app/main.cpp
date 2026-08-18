#include <CLI/CLI.hpp>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#if defined(__APPLE__) || defined(__linux__)
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

#include "app.hpp"

namespace fs = std::filesystem;

namespace {

std::optional<fs::path> optional_path(const std::string& value) {
  if (value.empty()) {
    return std::nullopt;
  }
  return fs::path(value);
}

pid_t spawn_process(const std::vector<std::string>& arguments,
                    const std::optional<fs::path>& log_path = std::nullopt) {
#if defined(__APPLE__) || defined(__linux__)
  std::vector<char*> argv;
  argv.reserve(arguments.size() + 1U);
  for (const std::string& argument : arguments) {
    argv.push_back(const_cast<char*>(argument.c_str()));
  }
  argv.push_back(nullptr);

  posix_spawn_file_actions_t file_actions;
  posix_spawn_file_actions_init(&file_actions);
  if (log_path) {
    posix_spawn_file_actions_addopen(&file_actions, STDOUT_FILENO,
                                     log_path->c_str(),
                                     O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&file_actions, STDOUT_FILENO,
                                     STDERR_FILENO);
  }

  pid_t child{};
  const int spawn_error = posix_spawnp(&child, argv.front(), &file_actions,
                                       nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&file_actions);
  if (spawn_error != 0) {
    throw std::runtime_error("could not start " + arguments.front());
  }
  return child;
#else
  static_cast<void>(arguments);
  static_cast<void>(log_path);
  throw std::runtime_error("uvd sitl currently supports macOS and Linux");
#endif
}

int exit_code_from_status(int status) {
#if defined(__APPLE__) || defined(__linux__)
  return WIFEXITED(status) ? WEXITSTATUS(status) : 128;
#else
  static_cast<void>(status);
  return 128;
#endif
}

int run_process(const std::vector<std::string>& arguments,
                const std::optional<fs::path>& log_path = std::nullopt) {
#if defined(__APPLE__) || defined(__linux__)
  const pid_t child = spawn_process(arguments, log_path);
  int status{};
  if (waitpid(child, &status, 0) < 0) {
    throw std::runtime_error("could not wait for " + arguments.front());
  }
  return exit_code_from_status(status);
#else
  static_cast<void>(arguments);
  static_cast<void>(log_path);
  throw std::runtime_error("uvd sitl currently supports macOS and Linux");
#endif
}

std::optional<fs::path> find_unreal_editor() {
  std::vector<fs::path> candidates;
  if (const char* configured = std::getenv("UVD_UNREAL_EDITOR")) {
    candidates.emplace_back(configured);
  }
#if defined(__APPLE__)
  candidates.emplace_back(
      "/Users/Shared/Epic "
      "Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/"
      "UnrealEditor");
  candidates.emplace_back(
      "/Users/Shared/Epic Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor");
  candidates.emplace_back(
      "/Applications/Epic "
      "Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/"
      "UnrealEditor");
  candidates.emplace_back(
      "/Applications/Epic Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor");
#elif defined(__linux__)
  if (const char* user_home = std::getenv("HOME")) {
    candidates.emplace_back(fs::path(user_home) /
                            "UnrealEngine/Engine/Binaries/Linux/UnrealEditor");
  }
  candidates.emplace_back(
      "/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor");
#endif
  for (const fs::path& candidate : candidates) {
    if (fs::is_regular_file(candidate)) {
#if defined(__APPLE__)
      const fs::path app_executable =
          candidate.parent_path() /
          "UnrealEditor.app/Contents/MacOS/UnrealEditor";
      if (candidate.filename() == "UnrealEditor" &&
          fs::is_regular_file(app_executable)) {
        return fs::absolute(app_executable);
      }
#endif
      return fs::absolute(candidate);
    }
  }
  return std::nullopt;
}

fs::path unreal_engine_root(const fs::path& editor) {
  fs::path candidate = editor.parent_path();
  while (!candidate.empty() && candidate != candidate.root_path()) {
    if (fs::is_regular_file(candidate / "Build/Build.version")) {
      return candidate;
    }
    candidate = candidate.parent_path();
  }
  return {};
}

std::optional<fs::path> find_cesium_plugin(const fs::path& editor) {
  const fs::path engine = unreal_engine_root(editor);
  const std::vector<fs::path> candidates{
      fs::path(UVD_SOURCE_DIR) /
          "unreal/Plugins/CesiumForUnreal/CesiumForUnreal.uplugin",
      engine / "Plugins/Marketplace/CesiumForUnreal/CesiumForUnreal.uplugin",
      engine / "Plugins/CesiumForUnreal/CesiumForUnreal.uplugin",
  };
  for (const fs::path& candidate : candidates) {
    if (fs::is_regular_file(candidate)) {
      return candidate;
    }
  }
  return std::nullopt;
}

bool supported_unreal_version(const fs::path& editor) {
  try {
    const uvd::app::Json version = uvd::app::parse_strict(
        unreal_engine_root(editor) / "Build/Build.version");
    return version.at("MajorVersion") == 5 && version.at("MinorVersion") == 8;
  } catch (const std::exception&) {
    return false;
  }
}

bool cesium_228_is_installed(const fs::path& editor) {
  const auto plugin = find_cesium_plugin(editor);
  if (!plugin) {
    return false;
  }
  try {
    const uvd::app::Json descriptor = uvd::app::parse_strict(*plugin);
    return descriptor.value("VersionName", std::string{}).starts_with("2.28");
  } catch (const std::exception&) {
    return false;
  }
}

fs::path unreal_build_script(const fs::path& editor) {
#if defined(__APPLE__)
  return unreal_engine_root(editor) / "Build/BatchFiles/Mac/Build.sh";
#elif defined(__linux__)
  return unreal_engine_root(editor) / "Build/BatchFiles/Linux/Build.sh";
#else
  return {};
#endif
}

void build_unreal_project(const fs::path& editor) {
  const fs::path project =
      fs::path(UVD_SOURCE_DIR) / "unreal/UnrealVehicleDynamics.uproject";
  const fs::path script = unreal_build_script(editor);
#if defined(__APPLE__)
  const std::string platform = "Mac";
#else
  const std::string platform = "Linux";
#endif
  const int result = run_process({script.string(), "UnrealEditor", platform,
                                  "Development", "-Project=" + project.string(),
                                  "-WaitMutex", "-NoHotReloadFromIDE"});
  if (result != 0) {
    throw std::runtime_error("Unreal project build failed");
  }
}

fs::path unreal_preflight(bool require_sitl_dependencies) {
  const auto editor = find_unreal_editor();
  std::vector<std::string> failures;
  const auto require = [&failures](bool passed, std::string_view name) {
    std::cout << (passed ? "pass " : "fail ") << name << '\n';
    if (!passed) {
      failures.emplace_back(name);
    }
  };

  require(fs::is_regular_file(fs::path(UVD_SOURCE_DIR) /
                              "unreal/UnrealVehicleDynamics.uproject"),
          "Unreal project");
  require(
      fs::is_regular_file(
          fs::path(UVD_SOURCE_DIR) /
          "unreal/Plugins/UnrealVehicleDynamics/ThirdParty/Eigen/Eigen/Core"),
      "staged Eigen headers");
  require(editor.has_value(), "Unreal Editor");
  require(editor && supported_unreal_version(*editor), "Unreal Engine 5.8");
  require(editor && fs::is_regular_file(unreal_build_script(*editor)),
          "Unreal build script");
#if defined(__APPLE__)
  require(run_process({"/usr/bin/xcodebuild", "-checkFirstLaunchStatus"},
                      "/dev/null") == 0,
          "selected and initialized Xcode");
  require(
      run_process({"/usr/bin/xcrun", "clang", "--version"}, "/dev/null") == 0,
      "Xcode compiler");
#endif
  if (editor && find_cesium_plugin(*editor)) {
    if (require_sitl_dependencies) {
      require(cesium_228_is_installed(*editor), "Cesium for Unreal 2.28");
    } else {
      std::cout << (cesium_228_is_installed(*editor) ? "pass " : "note ")
                << "Cesium for Unreal (not used by smoke run)\n";
    }
  } else if (require_sitl_dependencies) {
    require(false, "Cesium for Unreal 2.28");
  } else {
    std::cout << "skip Cesium for Unreal (not needed for smoke run)\n";
  }
  if (require_sitl_dependencies) {
    require(run_process({"docker", "version"}, "/dev/null") == 0, "Docker");
  }
  if (!failures.empty()) {
    std::ostringstream message;
    message << (require_sitl_dependencies ? "SITL" : "Unreal")
            << " preflight failed:";
    for (const std::string& failure : failures) {
      message << ' ' << failure << ';';
    }
    throw std::runtime_error(message.str());
  }
  return *editor;
}

void ensure_ardupilot_image() {
  constexpr std::string_view image = "uvd-ardupilot:e0652af";
  if (run_process({"docker", "image", "inspect", std::string(image)},
                  "/dev/null") == 0) {
    return;
  }
  const int result = run_process(
      {"docker", "build", "-t", std::string(image), "-f",
       std::string(UVD_SOURCE_DIR) + "/ardupilot/Dockerfile", UVD_SOURCE_DIR});
  if (result != 0) {
    throw std::runtime_error("could not build the pinned ArduPlane image");
  }
}

std::string controller_home(const uvd::app::RunConfig& run) {
  std::ostringstream home;
  home << std::setprecision(17) << run.origin_latitude_deg << ','
       << run.origin_longitude_deg << ',' << run.origin_altitude_msl_m << ','
       << run.starting_heading_deg;
  return home.str();
}

std::string safe_container_name(std::string run_id) {
  for (char& character : run_id) {
    if (!std::isalnum(static_cast<unsigned char>(character))) {
      character = '-';
    }
  }
#if defined(__APPLE__) || defined(__linux__)
  return "uvd-" + run_id + '-' + std::to_string(getpid());
#else
  return "uvd-" + run_id;
#endif
}

}  // namespace

namespace uvd::app {

void run_unreal(const RunConfig& run,
                const std::optional<std::filesystem::path>& output,
                bool preflight_only) {
  if (run.frontend != Frontend::kUnreal) {
    throw std::runtime_error("uvd unreal requires frontend 'unreal'");
  }
  if (run.controller) {
    throw std::runtime_error(
        "uvd unreal is scripted open loop; use uvd sitl for controller runs");
  }

  const fs::path editor = unreal_preflight(false);
  if (preflight_only) {
    return;
  }

  build_unreal_project(editor);
  Json manifest;
  const fs::path bundle = prepare_bundle(run, output, "unreal_smoke", manifest);
  const fs::path project =
      fs::path(UVD_SOURCE_DIR) / "unreal/UnrealVehicleDynamics.uproject";
  const fs::path unreal_log = bundle / "unreal.log";
  const pid_t unreal = spawn_process(
      {editor.string(), project.string(), "-game", "-log",
       "-abslog=" + unreal_log.string(), "-UvdRun=" + run.path.string(),
       "-UvdBundle=" + bundle.string()},
      bundle / "unreal-launcher.log");

  int status{};
  if (waitpid(unreal, &status, 0) < 0) {
    throw std::runtime_error("could not wait for Unreal Editor");
  }
  const int exit_code = exit_code_from_status(status);
  const fs::path manifest_path = bundle / "manifest.json";
  try {
    manifest = parse_strict(manifest_path);
  } catch (const std::exception&) {
    manifest["status"] = "failed";
    manifest["stop_reason"] = "missing_unreal_report";
  }
  manifest["launcher_exit_code"] = exit_code;
  if (exit_code != 0) {
    manifest["status"] = "failed";
    manifest["stop_reason"] = "unreal_exit_failure";
  }
  write_text(manifest_path, manifest.dump(2) + "\n");
  if (exit_code != 0 || manifest.value("status", std::string{}) != "complete") {
    throw std::runtime_error("Unreal smoke run failed; see " +
                             unreal_log.string());
  }
  std::cout << bundle << '\n';
}

void run_sitl(const RunConfig& run,
              const std::optional<std::filesystem::path>& output,
              bool preflight_only) {
  if (run.frontend != Frontend::kUnreal) {
    throw std::runtime_error("uvd sitl requires frontend 'unreal'");
  }
  if (!run.controller) {
    throw std::runtime_error("uvd sitl requires a controller configuration");
  }

  const fs::path editor = unreal_preflight(true);
  if (preflight_only) {
    return;
  }

  build_unreal_project(editor);
  ensure_ardupilot_image();
  Json manifest;
  const fs::path bundle = prepare_bundle(run, output, "sitl", manifest);
  fs::create_directories(bundle / "controller");

  const fs::path project =
      fs::path(UVD_SOURCE_DIR) / "unreal/UnrealVehicleDynamics.uproject";
  const fs::path unreal_log = bundle / "unreal.log";
  const pid_t unreal = spawn_process(
      {editor.string(), project.string(), "-game", "-log",
       "-abslog=" + unreal_log.string(), "-UvdRun=" + run.path.string(),
       "-UvdBundle=" + bundle.string()},
      bundle / "unreal-launcher.log");

  const std::string container = safe_container_name(run.run_id);
  std::vector<std::string> docker{
      "docker", "run", "--rm", "--name", container,
  };
#if defined(__linux__)
  docker.insert(docker.end(),
                {"--add-host", "host.docker.internal:host-gateway"});
#endif
  docker.insert(
      docker.end(),
      {"--tmpfs", "/root", "-e", "UVD_HOST_ADDRESS=host.docker.internal", "-e",
       "UVD_HOME=" + controller_home(run), "-e",
       "UVD_RATE_HZ=" + std::to_string(static_cast<unsigned int>(
                            std::llround(1.0 / run.dt))),
       "uvd-ardupilot:e0652af"});

  pid_t controller{};
  try {
    controller = spawn_process(docker, bundle / "controller/stdout.log");
  } catch (...) {
    kill(unreal, SIGTERM);
    waitpid(unreal, nullptr, 0);
    throw;
  }

  int status{};
  const pid_t finished = waitpid(-1, &status, 0);
  const int exit_code = exit_code_from_status(status);
  if (finished == unreal) {
    run_process({"docker", "stop", "--time", "3", container}, "/dev/null");
    waitpid(controller, nullptr, 0);
  } else {
    kill(unreal, SIGTERM);
    run_process({"docker", "stop", "--time", "3", container}, "/dev/null");
    waitpid(unreal, nullptr, 0);
    waitpid(controller, nullptr, 0);
  }

  manifest["status"] = exit_code == 0 ? "complete" : "failed";
  manifest["stop_reason"] =
      exit_code == 0 ? "launcher_completed" : "launcher_failed";
  manifest["launcher_exit_code"] = exit_code;
  write_text(bundle / "manifest.json", manifest.dump(2) + "\n");
  if (exit_code != 0) {
    throw std::runtime_error("SITL launcher exited with code " +
                             std::to_string(exit_code));
  }
}

}  // namespace uvd::app

int main(int argc, char** argv) {
  CLI::App app{"UnrealVehicleDynamics deterministic headless vehicle dynamics"};
  app.require_subcommand(1);

  std::string validate_path;
  std::string simulate_path;
  std::string trim_path;
  std::string linearize_path;
  std::string replay_path;
  std::string unreal_path;
  std::string sitl_path;
  std::string probe_aircraft_path;
  std::string left_comparison_path;
  std::string right_comparison_path;
  std::string simulate_output;
  std::string trim_output;
  std::string linearize_output;
  std::string comparison_output;
  std::string unreal_output;
  std::string sitl_output;
  bool unreal_preflight_only = false;
  bool sitl_preflight = false;

  auto* validate =
      app.add_subcommand("validate", "Validate an aircraft.json or run.json");
  validate->add_option("file", validate_path)->required();

  auto* simulate =
      app.add_subcommand("simulate", "Run a scripted headless simulation");
  simulate->add_option("run", simulate_path)->required();
  simulate->add_option("--output", simulate_output);

  auto* trim = app.add_subcommand(
      "trim", "Solve the straight-level 25 m/s operating point");
  trim->add_option("run", trim_path)->required();
  trim->add_option("--output", trim_output);

  auto* linearize =
      app.add_subcommand("linearize", "Linearize the discrete RK4 map at trim");
  linearize->add_option("run", linearize_path)->required();
  linearize->add_option("--output", linearize_output);

  auto* compare = app.add_subcommand(
      "compare", "Compare two run directories or signal CSV files");
  compare->add_option("left", left_comparison_path)->required();
  compare->add_option("right", right_comparison_path)->required();
  compare->add_option("--output", comparison_output);

  auto* replay = app.add_subcommand("replay", "Replay a saved run bundle");
  replay->add_option("run_directory", replay_path)->required();

  auto* unreal =
      app.add_subcommand("unreal", "Run a scripted open-loop Chaos case");
  unreal->add_option("run", unreal_path)->required();
  unreal->add_option("--output", unreal_output);
  unreal->add_flag("--preflight", unreal_preflight_only,
                   "Check Unreal host prerequisites without starting a run");

  auto* sitl = app.add_subcommand(
      "sitl", "Run Unreal with one managed ArduPlane controller");
  sitl->add_option("run", sitl_path)->required();
  sitl->add_option("--output", sitl_output);
  sitl->add_flag("--preflight", sitl_preflight,
                 "Check host prerequisites without starting a run");

  auto* model_probe = app.add_subcommand(
      "model-probe",
      "Evaluate aero-only JSONL cases for offline reference tools");
  model_probe->add_option("aircraft", probe_aircraft_path)->required();

  CLI11_PARSE(app, argc, argv);
  try {
    if (*validate) {
      uvd::app::run_validate(validate_path);
    } else if (*simulate) {
      const auto result = uvd::app::simulate(uvd::app::load_run(simulate_path),
                                             optional_path(simulate_output));
      std::cout << result.bundle << '\n';
    } else if (*trim) {
      uvd::app::run_trim(uvd::app::load_run(trim_path),
                         optional_path(trim_output));
    } else if (*linearize) {
      uvd::app::run_linearize(uvd::app::load_run(linearize_path),
                              optional_path(linearize_output));
    } else if (*compare) {
      uvd::app::run_compare(left_comparison_path, right_comparison_path,
                            optional_path(comparison_output));
    } else if (*replay) {
      uvd::app::run_replay(replay_path);
    } else if (*unreal) {
      uvd::app::run_unreal(uvd::app::load_run(unreal_path),
                           optional_path(unreal_output), unreal_preflight_only);
    } else if (*sitl) {
      uvd::app::run_sitl(uvd::app::load_run(sitl_path),
                         optional_path(sitl_output), sitl_preflight);
    } else if (*model_probe) {
      uvd::app::run_model_probe(probe_aircraft_path);
    }
  } catch (const std::exception& error) {
    std::cerr << "uvd: " << error.what() << '\n';
    return 2;
  }
  return 0;
}
