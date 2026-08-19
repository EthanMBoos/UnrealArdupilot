#include "config.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>

namespace fs = std::filesystem;

namespace uvd::cli {
namespace {

SurfaceMap surface_from_json(const Json& value) {
  return {.neutral_rad = value.at("neutral_rad"),
          .min_rad = value.at("min_rad"),
          .max_rad = value.at("max_rad"),
          .direction = value.at("direction")};
}

AerosondeParameters load_aircraft(const fs::path& path) {
  const Json json = load_json(path);
  AerosondeParameters parameters;
  parameters.model_id = json.at("model_id");
  parameters.mass_kg = json.at("mass_kg");

  const auto& inertia = json.at("inertia_kgm2");
  parameters.inertia_body_kgm2(0, 0) = inertia.at("jx");
  parameters.inertia_body_kgm2(1, 1) = inertia.at("jy");
  parameters.inertia_body_kgm2(2, 2) = inertia.at("jz");
  parameters.inertia_body_kgm2(0, 2) = -inertia.at("jxz").get<double>();
  parameters.inertia_body_kgm2(2, 0) = -inertia.at("jxz").get<double>();
  if (!(parameters.mass_kg > 0.0) ||
      !(parameters.inertia_body_kgm2.determinant() > 0.0)) {
    throw std::runtime_error("aircraft mass or inertia is invalid");
  }

  const auto& geometry = json.at("geometry");
  parameters.wing_area_m2 = geometry.at("wing_area_m2");
  parameters.span_m = geometry.at("span_m");
  parameters.chord_m = geometry.at("chord_m");
  parameters.oswald_efficiency = geometry.at("oswald_efficiency");

  const auto& aero = json.at("aerodynamics");
  auto& c = parameters.aero;
  c.C_L_0 = aero.at("CL_0");
  c.C_L_alpha = aero.at("CL_alpha");
  c.C_L_q = aero.at("CL_q");
  c.C_L_delta_e = aero.at("CL_de");
  c.C_D_parasitic = aero.at("CD_p");
  c.C_D_q = aero.at("CD_q");
  c.C_D_delta_e = aero.at("CD_de");
  c.C_Y_0 = aero.at("CY0");
  c.C_Y_beta = aero.at("CY_beta");
  c.C_Y_p = aero.at("CY_p");
  c.C_Y_r = aero.at("CY_r");
  c.C_Y_delta_a = aero.at("CY_da");
  c.C_Y_delta_r = aero.at("CY_dr");
  c.C_ell_0 = aero.at("Cl0");
  c.C_ell_beta = aero.at("Cl_beta");
  c.C_ell_p = aero.at("Cl_p");
  c.C_ell_r = aero.at("Cl_r");
  c.C_ell_delta_a = aero.at("Cl_da");
  c.C_ell_delta_r = aero.at("Cl_dr");
  c.C_m_0 = aero.at("Cm0");
  c.C_m_alpha = aero.at("Cm_alpha");
  c.C_m_q = aero.at("Cm_q");
  c.C_m_delta_e = aero.at("Cm_de");
  c.C_n_0 = aero.at("Cn0");
  c.C_n_beta = aero.at("Cn_beta");
  c.C_n_p = aero.at("Cn_p");
  c.C_n_r = aero.at("Cn_r");
  c.C_n_delta_a = aero.at("Cn_da");
  c.C_n_delta_r = aero.at("Cn_dr");
  c.alpha_stall_rad = aero.at("alpha0_rad");
  c.stall_blend_M = aero.at("lift_blend_M");

  const auto& propeller = json.at("propeller");
  auto& prop = parameters.propeller;
  prop.diameter_m = propeller.at("diameter_m");
  const double kv_rpm_per_volt = propeller.at("KV_rpm_per_volt");
  prop.motor_torque_constant_Nm_per_A =
      60.0 / (2.0 * std::numbers::pi * kv_rpm_per_volt);
  prop.resistance_ohm = propeller.at("resistance_ohm");
  prop.no_load_current_A = propeller.at("no_load_current_amp");
  prop.max_voltage_V = propeller.at("max_voltage_v");
  prop.C_Q = {.x2 = propeller.at("CQ").at(0),
              .x1 = propeller.at("CQ").at(1),
              .x0 = propeller.at("CQ").at(2)};
  prop.C_T = {.x2 = propeller.at("CT").at(0),
              .x1 = propeller.at("CT").at(1),
              .x0 = propeller.at("CT").at(2)};
  prop.position_body_m = vector3_from_json(propeller.at("position_body_m"));
  prop.advance_ratio_min = propeller.at("advance_ratio_range").at(0);
  prop.advance_ratio_max = propeller.at("advance_ratio_range").at(1);

  const auto& actuators = json.at("actuators");
  parameters.actuator = {
      .aileron = surface_from_json(actuators.at("aileron")),
      .elevator = surface_from_json(actuators.at("elevator")),
      .rudder = surface_from_json(actuators.at("rudder"))};
  return parameters;
}

}  // namespace

Json load_json(const fs::path& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot read " + path.string());
  }
  Json value;
  stream >> value;
  return value;
}

void write_json(const fs::path& path, const Json& value) {
  if (!path.parent_path().empty()) {
    fs::create_directories(path.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot write " + path.string());
  }
  stream << value.dump(2) << '\n';
}

Vector3 vector3_from_json(const Json& value) {
  return {value.at(0).get<double>(), value.at(1).get<double>(),
          value.at(2).get<double>()};
}

RigidBodyState state_from_json(const Json& value) {
  const auto& q = value.at("q_body_to_ned");
  return {.position_ned_m = vector3_from_json(value.at("position_ned_m")),
          .q_body_to_ned =
              canonicalize(Quaternion{q.at(0), q.at(1), q.at(2), q.at(3)}),
          .velocity_body_mps = vector3_from_json(value.at("velocity_body_mps")),
          .omega_body_radps = vector3_from_json(value.at("omega_body_radps"))};
}

Json to_json(const Vector3& value) {
  return Json::array({value.x(), value.y(), value.z()});
}

Json to_json(const Quaternion& value) {
  return Json::array({value.w(), value.x(), value.y(), value.z()});
}

Json to_json(const RigidBodyState& state) {
  return {{"position_ned_m", to_json(state.position_ned_m)},
          {"q_body_to_ned", to_json(state.q_body_to_ned)},
          {"velocity_body_mps", to_json(state.velocity_body_mps)},
          {"omega_body_radps", to_json(state.omega_body_radps)}};
}

Json to_json(const AircraftCommand& command) {
  return {{"aileron", command.aileron},
          {"elevator", command.elevator},
          {"rudder", command.rudder},
          {"throttle", command.throttle}};
}

Json to_json(const AircraftEffectorState& effectors) {
  return {{"aileron_rad", effectors.aileron_rad},
          {"elevator_rad", effectors.elevator_rad},
          {"rudder_rad", effectors.rudder_rad},
          {"throttle", effectors.throttle}};
}

RunConfig load_run(const fs::path& raw_path) {
  RunConfig run;
  run.path = fs::absolute(raw_path).lexically_normal();
  const Json json = load_json(run.path);
  run.fixed_dt_s = json.at("fixed_dt_s");
  if (!(run.fixed_dt_s > 0.0)) {
    throw std::runtime_error("fixed_dt_s must be positive");
  }
  run.aircraft_path =
      (run.path.parent_path() / json.at("aircraft").get<std::string>())
          .lexically_normal();
  run.aircraft = load_aircraft(run.aircraft_path);
  run.origin_altitude_msl_m = json.at("origin").at("altitude_msl_m");
  run.wind_ned_mps = vector3_from_json(json.at("wind_ned_mps"));
  run.initial_state = state_from_json(json.at("initial_state"));
  const auto& trim = json.at("trim");
  run.trim = {.aileron = trim.at("aileron"),
              .elevator = trim.at("elevator"),
              .rudder = trim.at("rudder"),
              .throttle = trim.at("throttle")};
  return run;
}

}  // namespace uvd::cli
