#pragma once

#include <cstdint>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "uvd/core.hpp"

namespace uvd::app {

using Json = nlohmann::json;

enum class InputBoundary {
  kAircraftCommand,
  kAircraftEffector,
};

enum class ScheduleMode {
  kAbsolute,
  kOffsetFromTrim,
};

enum class FinalAction {
  kHold,
  kStop,
};

struct ScheduledInput {
  std::optional<double> aileron;
  std::optional<double> elevator;
  std::optional<double> rudder;
  std::optional<double> throttle;

  [[nodiscard]] bool complete() const noexcept;
  void apply_to(AircraftCommand& input) const noexcept;
};

struct ScheduleEntry {
  std::uint64_t tick{};
  ScheduledInput values;
};

struct LoadedAircraft {
  std::filesystem::path path;
  Json json;
  AerosondeParameters parameters;
};

struct RunConfig {
  std::filesystem::path path;
  Json json;
  std::string run_id;
  LoadedAircraft aircraft;
  double dt{};
  double origin_altitude_msl_m{};
  Vector3 wind_ned_mps = Vector3::Zero();
  RigidBodyState initial_state{};
  InputBoundary input_boundary = InputBoundary::kAircraftCommand;
  ScheduleMode schedule_mode = ScheduleMode::kAbsolute;
  std::optional<std::filesystem::path> trim_path;
  std::vector<ScheduleEntry> schedule;
  FinalAction final_action = FinalAction::kHold;
  std::uint64_t final_tick{};
  std::filesystem::path output_root;
};

struct SimulationResult {
  std::filesystem::path bundle;
  RigidBodyState final_state;
  Json manifest;
};

[[nodiscard]] std::string read_text(const std::filesystem::path& path);
void write_text(const std::filesystem::path& path, std::string_view text);
[[nodiscard]] Json parse_strict(const std::filesystem::path& path);

[[nodiscard]] Vector3 vector3_from_json(const Json& value);
[[nodiscard]] RigidBodyState state_from_json(const Json& value);
[[nodiscard]] Json to_json(const Vector3& value);
[[nodiscard]] Json to_json(const Quaternion& value);
[[nodiscard]] Json to_json(const RigidBodyState& state);
[[nodiscard]] Json to_json(const AircraftCommand& command);
[[nodiscard]] Json to_json(const AircraftEffectorState& effectors);

[[nodiscard]] std::string_view input_boundary_name(
    InputBoundary boundary) noexcept;
[[nodiscard]] LoadedAircraft load_aircraft(
    const std::filesystem::path& raw_path);
[[nodiscard]] RunConfig load_run(const std::filesystem::path& raw_path);
void validate_input(const AircraftCommand& input, InputBoundary boundary);
[[nodiscard]] AircraftEffectorState effectors_for_input(
    const RunConfig& run, const AircraftCommand& input);

[[nodiscard]] std::filesystem::path prepare_bundle(
    const RunConfig& run,
    const std::optional<std::filesystem::path>& override_path,
    std::string_view operation, Json& manifest);

void run_validate(const std::filesystem::path& path);
[[nodiscard]] SimulationResult simulate(
    const RunConfig& run, const std::optional<std::filesystem::path>& output,
    bool replay = false);
void run_trim(const RunConfig& run,
              const std::optional<std::filesystem::path>& output);
void run_linearize(const RunConfig& run,
                   const std::optional<std::filesystem::path>& output);
void run_compare(const std::filesystem::path& left,
                 const std::filesystem::path& right,
                 const std::optional<std::filesystem::path>& output);
void run_replay(const std::filesystem::path& bundle_path);
void run_model_probe(const std::filesystem::path& aircraft_path);

}  // namespace uvd::app
