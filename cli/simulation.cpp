#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "config.hpp"

namespace uvd::cli {
namespace {

AircraftEffectorState effectors_from_json(const Json& value) {
  return {.aileron_rad = value.at("aileron_rad"),
          .elevator_rad = value.at("elevator_rad"),
          .rudder_rad = value.at("rudder_rad"),
          .throttle = value.value("throttle", 0.0)};
}

AircraftCommand command_from_json(const Json& value) {
  return {.aileron = value.at("aileron"),
          .elevator = value.at("elevator"),
          .rudder = value.at("rudder"),
          .throttle = value.at("throttle")};
}

void write_csv_row(std::ostream& stream, double time_s,
                   const RigidBodyState& state) {
  stream << time_s << ',' << state.position_ned_m.x() << ','
         << state.position_ned_m.y() << ',' << state.position_ned_m.z() << ','
         << state.q_body_to_ned.w() << ',' << state.q_body_to_ned.x() << ','
         << state.q_body_to_ned.y() << ',' << state.q_body_to_ned.z() << ','
         << state.velocity_body_mps.x() << ',' << state.velocity_body_mps.y()
         << ',' << state.velocity_body_mps.z() << ','
         << state.omega_body_radps.x() << ',' << state.omega_body_radps.y()
         << ',' << state.omega_body_radps.z() << '\n';
}

}  // namespace

Json simulate(const RunConfig& run, const SimulationOptions& options) {
  const double dt_s = options.dt_s.value_or(run.fixed_dt_s);
  if (!(dt_s > 0.0) || !(options.duration_s >= 0.0)) {
    throw std::runtime_error("duration must be nonnegative and dt positive");
  }
  const auto step_count =
      static_cast<std::uint64_t>(std::llround(options.duration_s / dt_s));
  const AircraftEffectorState effectors =
      map_command(run.trim, run.aircraft.actuator);
  RigidBodyState state = run.initial_state;

  std::ofstream csv;
  if (options.output) {
    if (!options.output->parent_path().empty()) {
      std::filesystem::create_directories(options.output->parent_path());
    }
    csv.open(*options.output);
    if (!csv) {
      throw std::runtime_error("cannot write " + options.output->string());
    }
    csv << std::setprecision(17)
        << "time_s,north_m,east_m,down_m,qw,qx,qy,qz,u_mps,v_mps,w_mps,"
           "p_radps,q_radps,r_radps\n";
    write_csv_row(csv, 0.0, state);
  }

  for (std::uint64_t step = 0; step < step_count; ++step) {
    state = step_aerosonde_rk4(state, effectors, run.origin_altitude_msl_m,
                               run.wind_ned_mps, run.aircraft, dt_s);
    if (!is_finite(state)) {
      throw std::runtime_error("simulation became nonfinite at step " +
                               std::to_string(step + 1));
    }
    if (csv) {
      write_csv_row(csv, static_cast<double>(step + 1) * dt_s, state);
    }
  }

  Json result = {{"kind", "headless_simulation"},
                 {"model_id", run.aircraft.model_id},
                 {"integrator", "rk4"},
                 {"dt_s", dt_s},
                 {"steps", step_count},
                 {"duration_s", static_cast<double>(step_count) * dt_s},
                 {"command", to_json(run.trim)},
                 {"effectors", to_json(effectors)},
                 {"final_state", to_json(state)}};
  if (options.output) {
    result["trajectory_csv"] = options.output->string();
  }
  return result;
}

Json evaluate(const RunConfig& run, const std::optional<Json>& input) {
  const Json empty = Json::object();
  const Json& value = input ? *input : empty;
  const RigidBodyState state = value.contains("state")
                                   ? state_from_json(value.at("state"))
                                   : run.initial_state;
  AircraftEffectorState effectors =
      map_command(run.trim, run.aircraft.actuator);
  if (value.contains("effectors")) {
    effectors = effectors_from_json(value.at("effectors"));
  } else if (value.contains("command")) {
    effectors = map_command(command_from_json(value.at("command")),
                            run.aircraft.actuator);
  }
  const Vector3 wind = value.contains("wind_ned_mps")
                           ? vector3_from_json(value.at("wind_ned_mps"))
                           : run.wind_ned_mps;
  AtmosphereSnapshot atmosphere = evaluate_isa(
      value.value("altitude_msl_m", run.origin_altitude_msl_m), wind);
  if (value.contains("density_kgpm3")) {
    atmosphere.density_kgpm3 = value.at("density_kgpm3");
  }
  const AircraftModelOutput model =
      evaluate_aerosonde(state, effectors, atmosphere, run.aircraft);
  const auto& air = model.aerodynamics.air_data;
  const auto& c = model.aerodynamics.coefficients;
  return {
      {"kind", "model_evaluation"},
      {"model_id", run.aircraft.model_id},
      {"state", to_json(state)},
      {"effectors", to_json(effectors)},
      {"air_data",
       {{"tas_mps", air.true_airspeed_mps},
        {"alpha_rad", air.alpha_rad},
        {"beta_rad", air.beta_rad},
        {"qbar_pa", air.dynamic_pressure_Pa}}},
      {"coefficients",
       Json::array({c.C_D, c.C_L, c.C_Y, c.C_ell, c.C_m, c.C_n})},
      {"aerodynamic_force_n", to_json(model.aerodynamics.wrench.force_body_N)},
      {"aerodynamic_moment_nm",
       to_json(model.aerodynamics.wrench.moment_body_Nm)},
      {"propeller_force_n", to_json(model.propulsion.wrench.force_body_N)},
      {"propeller_moment_nm", to_json(model.propulsion.wrench.moment_body_Nm)},
      {"total_force_n", to_json(model.total_wrench.force_body_N)},
      {"total_moment_nm", to_json(model.total_wrench.moment_body_Nm)},
      {"valid", model.valid}};
}

}  // namespace uvd::cli
