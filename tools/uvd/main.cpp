#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

#include "app.hpp"

namespace fs = std::filesystem;

namespace {

std::optional<fs::path> optional_path(const std::string& value) {
  if (value.empty()) {
    return std::nullopt;
  }
  return fs::path(value);
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"UnrealVehicleDynamics deterministic headless vehicle dynamics"};
  app.require_subcommand(1);

  std::string validate_path;
  std::string simulate_path;
  std::string trim_path;
  std::string linearize_path;
  std::string replay_path;
  std::string probe_aircraft_path;
  std::string left_comparison_path;
  std::string right_comparison_path;
  std::string simulate_output;
  std::string trim_output;
  std::string linearize_output;
  std::string comparison_output;

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
    } else if (*model_probe) {
      uvd::app::run_model_probe(probe_aircraft_path);
    }
  } catch (const std::exception& error) {
    std::cerr << "uvd: " << error.what() << '\n';
    return 2;
  }
  return 0;
}
