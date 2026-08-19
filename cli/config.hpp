#ifndef UNREALVEHICLEDYNAMICS_CLI_CONFIG_HPP_
#define UNREALVEHICLEDYNAMICS_CLI_CONFIG_HPP_

#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "uvd/core.hpp"

namespace uvd::cli {

using Json = nlohmann::json;

struct RunConfig {
  std::filesystem::path path;
  std::filesystem::path aircraft_path;
  AerosondeParameters aircraft;
  double fixed_dt_s{};
  double origin_altitude_msl_m{};
  Vector3 wind_ned_mps = Vector3::Zero();
  RigidBodyState initial_state;
  AircraftCommand trim;
};

struct SimulationOptions {
  double duration_s{10.0};
  std::optional<double> dt_s;
  std::optional<std::filesystem::path> output;
};

[[nodiscard]] RunConfig load_run(const std::filesystem::path& path);
[[nodiscard]] Json load_json(const std::filesystem::path& path);
void write_json(const std::filesystem::path& path, const Json& value);

[[nodiscard]] Vector3 vector3_from_json(const Json& value);
[[nodiscard]] RigidBodyState state_from_json(const Json& value);
[[nodiscard]] Json to_json(const Vector3& value);
[[nodiscard]] Json to_json(const Quaternion& value);
[[nodiscard]] Json to_json(const RigidBodyState& state);
[[nodiscard]] Json to_json(const AircraftCommand& command);
[[nodiscard]] Json to_json(const AircraftEffectorState& effectors);

[[nodiscard]] Json simulate(const RunConfig& run,
                            const SimulationOptions& options);
[[nodiscard]] Json evaluate(const RunConfig& run,
                            const std::optional<Json>& input);
[[nodiscard]] Json trim(const RunConfig& run);
[[nodiscard]] Json linearize(const RunConfig& run);

}  // namespace uvd::cli

#endif  // UNREALVEHICLEDYNAMICS_CLI_CONFIG_HPP_
