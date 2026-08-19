#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <nlohmann/json-schema.hpp>
#include <numbers>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "app.hpp"

namespace fs = std::filesystem;

namespace uvd::app {
namespace {

fs::path source_root() { return fs::path(UVD_SOURCE_DIR); }

void validate_schema(const Json& instance, const fs::path& schema_path) {
  nlohmann::json_schema::json_validator validator;
  validator.set_root_schema(parse_strict(schema_path));
  validator.validate(instance);
}

Quaternion quaternion_from_json(const Json& value) {
  return canonicalize(
      Quaternion{value.at(0).get<double>(), value.at(1).get<double>(),
                 value.at(2).get<double>(), value.at(3).get<double>()});
}

SurfaceMap surface_from_json(const Json& value) {
  return SurfaceMap{
      .neutral_rad = value.at("neutral_rad"),
      .min_rad = value.at("min_rad"),
      .max_rad = value.at("max_rad"),
      .direction = value.at("direction"),
  };
}

InputBoundary parse_input_boundary(std::string_view name) {
  if (name == "aircraft_command") {
    return InputBoundary::kAircraftCommand;
  }
  if (name == "aircraft_effector") {
    return InputBoundary::kAircraftEffector;
  }
  throw std::runtime_error("unsupported controls.input_boundary");
}

Frontend parse_frontend(std::string_view name) {
  if (name == "headless") {
    return Frontend::kHeadless;
  }
  if (name == "unreal") {
    return Frontend::kUnreal;
  }
  throw std::runtime_error("unsupported frontend");
}

std::string_view frontend_name(Frontend frontend) noexcept {
  return frontend == Frontend::kHeadless ? "headless" : "unreal";
}

MotionSolver parse_motion_solver(std::string_view name) {
  if (name == "rk4") {
    return MotionSolver::kRk4;
  }
  if (name == "chaos") {
    return MotionSolver::kChaos;
  }
  throw std::runtime_error("unsupported clock.motion_solver");
}

std::string_view motion_solver_name(MotionSolver solver) noexcept {
  return solver == MotionSolver::kRk4 ? "rk4" : "chaos";
}

ScheduleMode parse_schedule_mode(std::string_view name) {
  if (name == "absolute") {
    return ScheduleMode::kAbsolute;
  }
  if (name == "offset_from_trim") {
    return ScheduleMode::kOffsetFromTrim;
  }
  throw std::runtime_error("unsupported controls.mode");
}

FinalAction parse_final_action(std::string_view name) {
  if (name == "hold") {
    return FinalAction::kHold;
  }
  if (name == "stop") {
    return FinalAction::kStop;
  }
  throw std::runtime_error("unsupported controls.final");
}

ScheduledInput scheduled_input_from_json(const Json& values) {
  ScheduledInput input;
  if (values.contains("aileron")) {
    input.aileron = values.at("aileron").get<double>();
  }
  if (values.contains("elevator")) {
    input.elevator = values.at("elevator").get<double>();
  }
  if (values.contains("rudder")) {
    input.rudder = values.at("rudder").get<double>();
  }
  if (values.contains("throttle")) {
    input.throttle = values.at("throttle").get<double>();
  }
  return input;
}

std::string timestamp_id() {
  const auto value =
      std::chrono::system_clock::now().time_since_epoch().count();
  return std::to_string(value);
}

Json base_manifest(const RunConfig& run, std::string_view operation) {
  Json result = {
      {"schema_version", 1},
      {"operation", operation},
      {"run_id", run.run_id},
      {"frontend", frontend_name(run.frontend)},
      {"model_id", run.aircraft.parameters.model_id},
      {"fixed_dt_s", run.dt},
      {"motion_solver", motion_solver_name(run.motion_solver)},
      {"status", "running"},
      {"git_commit", UVD_GIT_COMMIT},
      {"git_dirty", static_cast<bool>(UVD_GIT_DIRTY)},
      {"reference_class", UVD_GIT_DIRTY ? "exploratory" : "reference"},
      {"warnings", Json::array()}};
#if defined(__APPLE__)
  result["platform"] = "macOS";
#elif defined(__linux__)
  result["platform"] = "Linux";
#else
  result["platform"] = "unknown";
#endif
  result["compiler"] =
#if defined(__clang__)
      std::string("Clang ") + __clang_version__;
#elif defined(__GNUC__)
      std::string("GCC ") + __VERSION__;
#else
      "unknown";
#endif
  return result;
}

}  // namespace

bool ScheduledInput::complete() const noexcept {
  return aileron && elevator && rudder && throttle;
}

void ScheduledInput::apply_to(AircraftCommand& input) const noexcept {
  if (aileron) {
    input.aileron = *aileron;
  }
  if (elevator) {
    input.elevator = *elevator;
  }
  if (rudder) {
    input.rudder = *rudder;
  }
  if (throttle) {
    input.throttle = *throttle;
  }
}

std::string read_text(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot read file: " + path.string());
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  if (!stream.good() && !stream.eof()) {
    throw std::runtime_error("failed reading: " + path.string());
  }
  return contents.str();
}

void write_text(const fs::path& path, std::string_view text) {
  fs::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("cannot write file: " + path.string());
  }
  stream.write(text.data(), static_cast<std::streamsize>(text.size()));
  if (!stream) {
    throw std::runtime_error("failed writing: " + path.string());
  }
}

Json parse_strict(const fs::path& path) {
  const std::string text = read_text(path);
  std::vector<std::unordered_set<std::string>> object_keys;
  bool duplicate = false;
  std::string duplicate_name;
  auto callback = [&](int, Json::parse_event_t event, Json& parsed) {
    if (event == Json::parse_event_t::object_start) {
      object_keys.emplace_back();
    }
    if (event == Json::parse_event_t::key && !object_keys.empty()) {
      const std::string key = parsed.get<std::string>();
      if (!object_keys.back().insert(key).second) {
        duplicate = true;
        duplicate_name = key;
      }
    }
    if (event == Json::parse_event_t::object_end && !object_keys.empty()) {
      object_keys.pop_back();
    }
    return true;
  };
  Json result = Json::parse(text, callback, true, false);
  if (duplicate) {
    throw std::runtime_error("duplicate JSON key '" + duplicate_name + "' in " +
                             path.string());
  }
  std::function<void(const Json&)> finite = [&](const Json& value) {
    if (value.is_number_float() && !std::isfinite(value.get<double>())) {
      throw std::runtime_error("nonfinite JSON number in " + path.string());
    }
    if (value.is_array()) {
      for (const auto& child : value) {
        finite(child);
      }
    }
    if (value.is_object()) {
      for (const auto& [name, child] : value.items()) {
        static_cast<void>(name);
        finite(child);
      }
    }
  };
  finite(result);
  return result;
}

Vector3 vector3_from_json(const Json& value) {
  return {value.at(0).get<double>(), value.at(1).get<double>(),
          value.at(2).get<double>()};
}

RigidBodyState state_from_json(const Json& value) {
  return RigidBodyState{
      .position_ned_m = vector3_from_json(value.at("position_ned_m")),
      .q_body_to_ned = quaternion_from_json(value.at("q_body_to_ned")),
      .velocity_body_mps = vector3_from_json(value.at("velocity_body_mps")),
      .omega_body_radps = vector3_from_json(value.at("omega_body_radps")),
  };
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

std::string_view input_boundary_name(InputBoundary boundary) noexcept {
  switch (boundary) {
    case InputBoundary::kAircraftCommand:
      return "aircraft_command";
    case InputBoundary::kAircraftEffector:
      return "aircraft_effector";
  }
  return "unknown";
}

LoadedAircraft load_aircraft(const fs::path& raw_path) {
  const fs::path path = fs::weakly_canonical(raw_path);
  Json json = parse_strict(path);
  validate_schema(json, source_root() / "schemas/aircraft.schema.json");

  AerosondeParameters parameters;
  parameters.model_id = json.at("model_id");
  parameters.mass_kg = json.at("mass_kg");

  const auto& inertia = json.at("inertia_kgm2");
  const double jx = inertia.at("jx");
  const double jy = inertia.at("jy");
  const double jz = inertia.at("jz");
  const double jxz = inertia.at("jxz");
  parameters.inertia_body_kgm2(0, 0) = jx;
  parameters.inertia_body_kgm2(1, 1) = jy;
  parameters.inertia_body_kgm2(2, 2) = jz;
  parameters.inertia_body_kgm2(0, 2) = -jxz;
  parameters.inertia_body_kgm2(2, 0) = -jxz;
  if (!(parameters.inertia_body_kgm2.determinant() > 0.0 && jx > 0.0 &&
        jy > 0.0 && jx * jz - jxz * jxz > 0.0)) {
    throw std::runtime_error("aircraft inertia is not positive definite");
  }

  const auto& geometry = json.at("geometry");
  parameters.wing_area_m2 = geometry.at("wing_area_m2");
  parameters.span_m = geometry.at("span_m");
  parameters.chord_m = geometry.at("chord_m");
  parameters.oswald_efficiency = geometry.at("oswald_efficiency");
  const double derived_aspect_ratio =
      parameters.span_m * parameters.span_m / parameters.wing_area_m2;
  const double configured_aspect_ratio = geometry.at("aspect_ratio");
  if (std::abs(configured_aspect_ratio - derived_aspect_ratio) >
      1e-12 * derived_aspect_ratio) {
    throw std::runtime_error(
        "geometry.aspect_ratio does not equal span^2/area");
  }

  const auto& aerodynamics = json.at("aerodynamics");
  auto& coefficients = parameters.aero;
  coefficients.C_L_0 = aerodynamics.at("CL_0");
  coefficients.C_L_alpha = aerodynamics.at("CL_alpha");
  coefficients.C_L_q = aerodynamics.at("CL_q");
  coefficients.C_L_delta_e = aerodynamics.at("CL_de");
  coefficients.C_D_parasitic = aerodynamics.at("CD_p");
  coefficients.C_D_q = aerodynamics.at("CD_q");
  coefficients.C_D_delta_e = aerodynamics.at("CD_de");
  coefficients.C_Y_0 = aerodynamics.at("CY0");
  coefficients.C_Y_beta = aerodynamics.at("CY_beta");
  coefficients.C_Y_p = aerodynamics.at("CY_p");
  coefficients.C_Y_r = aerodynamics.at("CY_r");
  coefficients.C_Y_delta_a = aerodynamics.at("CY_da");
  coefficients.C_Y_delta_r = aerodynamics.at("CY_dr");
  coefficients.C_ell_0 = aerodynamics.at("Cl0");
  coefficients.C_ell_beta = aerodynamics.at("Cl_beta");
  coefficients.C_ell_p = aerodynamics.at("Cl_p");
  coefficients.C_ell_r = aerodynamics.at("Cl_r");
  coefficients.C_ell_delta_a = aerodynamics.at("Cl_da");
  coefficients.C_ell_delta_r = aerodynamics.at("Cl_dr");
  coefficients.C_m_0 = aerodynamics.at("Cm0");
  coefficients.C_m_alpha = aerodynamics.at("Cm_alpha");
  coefficients.C_m_q = aerodynamics.at("Cm_q");
  coefficients.C_m_delta_e = aerodynamics.at("Cm_de");
  coefficients.C_n_0 = aerodynamics.at("Cn0");
  coefficients.C_n_beta = aerodynamics.at("Cn_beta");
  coefficients.C_n_p = aerodynamics.at("Cn_p");
  coefficients.C_n_r = aerodynamics.at("Cn_r");
  coefficients.C_n_delta_a = aerodynamics.at("Cn_da");
  coefficients.C_n_delta_r = aerodynamics.at("Cn_dr");
  coefficients.alpha_stall_rad = aerodynamics.at("alpha0_rad");
  coefficients.stall_blend_M = aerodynamics.at("lift_blend_M");

  const auto& propeller = json.at("propeller");
  auto& propeller_parameters = parameters.propeller;
  propeller_parameters.diameter_m = propeller.at("diameter_m");
  const double kv_rpm_per_V = propeller.at("KV_rpm_per_volt");
  // In SI units the motor torque constant (N*m/A) is the reciprocal speed
  // constant (V*s/rad).
  propeller_parameters.motor_torque_constant_Nm_per_A =
      (1.0 / kv_rpm_per_V) * 60.0 / (2.0 * std::numbers::pi);
  propeller_parameters.resistance_ohm = propeller.at("resistance_ohm");
  propeller_parameters.no_load_current_A = propeller.at("no_load_current_amp");
  propeller_parameters.max_voltage_V = propeller.at("max_voltage_v");
  propeller_parameters.C_Q = {
      .x2 = propeller.at("CQ").at(0),
      .x1 = propeller.at("CQ").at(1),
      .x0 = propeller.at("CQ").at(2),
  };
  propeller_parameters.C_T = {
      .x2 = propeller.at("CT").at(0),
      .x1 = propeller.at("CT").at(1),
      .x0 = propeller.at("CT").at(2),
  };
  propeller_parameters.position_body_m =
      vector3_from_json(propeller.at("position_body_m"));
  propeller_parameters.advance_ratio_min =
      propeller.at("advance_ratio_range").at(0);
  propeller_parameters.advance_ratio_max =
      propeller.at("advance_ratio_range").at(1);
  if (propeller_parameters.advance_ratio_min >=
      propeller_parameters.advance_ratio_max) {
    throw std::runtime_error("invalid propeller advance-ratio range");
  }

  parameters.actuator = {
      .aileron = surface_from_json(json.at("actuators").at("aileron")),
      .elevator = surface_from_json(json.at("actuators").at("elevator")),
      .rudder = surface_from_json(json.at("actuators").at("rudder")),
  };
  const auto surface_is_valid = [](const SurfaceMap& surface) {
    return surface.min_rad < surface.neutral_rad &&
           surface.neutral_rad < surface.max_rad &&
           std::abs(surface.direction) == 1.0;
  };
  if (!surface_is_valid(parameters.actuator.aileron) ||
      !surface_is_valid(parameters.actuator.elevator) ||
      !surface_is_valid(parameters.actuator.rudder)) {
    throw std::runtime_error("invalid actuator surface range");
  }

  std::set<std::string> functions;
  std::set<int> channels;
  for (const auto& entry : json.at("channel_map")) {
    const int channel = entry.at("channel");
    const std::string function = entry.at("function");
    if (!channels.insert(channel).second ||
        !functions.insert(function).second) {
      throw std::runtime_error("duplicate channel map channel or function");
    }
    const int minimum = entry.at("pwm_min");
    const int trim = entry.at("pwm_trim");
    const int maximum = entry.at("pwm_max");
    if (!(minimum <= trim && trim <= maximum && minimum < maximum)) {
      throw std::runtime_error("invalid PWM range");
    }
  }
  if (functions !=
      std::set<std::string>{"aileron", "elevator", "rudder", "throttle"}) {
    throw std::runtime_error(
        "channel map must define all four functions exactly once");
  }

  return LoadedAircraft{
      .path = path,
      .json = std::move(json),
      .parameters = std::move(parameters),
  };
}

RunConfig load_run(const fs::path& raw_path) {
  RunConfig run;
  run.path = fs::weakly_canonical(raw_path);
  run.json = parse_strict(run.path);
  validate_schema(run.json, source_root() / "schemas/run.schema.json");
  run.run_id = run.json.at("run_id");
  run.frontend = parse_frontend(run.json.at("frontend").get<std::string>());
  run.dt = run.json.at("clock").at("fixed_dt_s");
  run.motion_solver = parse_motion_solver(
      run.json.at("clock").at("motion_solver").get<std::string>());

  const fs::path aircraft_path =
      run.path.parent_path() /
      run.json.at("aircraft").at("path").get<std::string>();
  run.aircraft = load_aircraft(aircraft_path);

  const auto& world = run.json.at("world");
  run.origin_latitude_deg = world.at("origin_latitude_deg");
  run.origin_longitude_deg = world.at("origin_longitude_deg");
  run.origin_altitude_msl_m = world.at("origin_altitude_msl_m");
  run.starting_heading_deg = world.at("starting_heading_deg");
  run.geoid_undulation_m = world.value("geoid_undulation_m", 0.0);
  run.wind_ned_mps =
      vector3_from_json(run.json.at("atmosphere").at("wind_ned_mps"));
  run.initial_state = state_from_json(run.json.at("initial_state"));

  const auto& controls = run.json.at("controls");
  run.input_boundary =
      parse_input_boundary(controls.at("input_boundary").get<std::string>());
  run.schedule_mode =
      parse_schedule_mode(controls.at("mode").get<std::string>());
  run.final_action =
      parse_final_action(controls.at("final").get<std::string>());
  if (controls.contains("trim_path")) {
    run.trim_path = fs::weakly_canonical(
        run.path.parent_path() / controls.at("trim_path").get<std::string>());
  }

  std::optional<std::uint64_t> previous_tick;
  std::optional<std::uint64_t> previous_arrival_tick;
  for (const auto& item : controls.at("schedule")) {
    const std::uint64_t apply_tick = item.at("apply_tick");
    ScheduleEntry entry{
        .tick = apply_tick,
        .arrival_tick = item.value("arrival_tick", apply_tick),
        .values = scheduled_input_from_json(item.at("values")),
    };
    if (previous_tick && entry.tick <= *previous_tick) {
      throw std::runtime_error("schedule ticks must be strictly increasing");
    }
    previous_tick = entry.tick;
    if (entry.arrival_tick < entry.tick) {
      throw std::runtime_error("schedule arrival_tick precedes apply_tick");
    }
    if (previous_arrival_tick && entry.arrival_tick <= *previous_arrival_tick) {
      throw std::runtime_error(
          "schedule arrival ticks must be strictly increasing");
    }
    previous_arrival_tick = entry.arrival_tick;
    if (item.contains("time_s") &&
        std::abs(item.at("time_s").get<double>() -
                 static_cast<double>(entry.tick) * run.dt) > 1e-12) {
      throw std::runtime_error("schedule time_s is off the fixed tick grid");
    }
    run.schedule.push_back(std::move(entry));
  }
  if (run.frontend == Frontend::kHeadless) {
    for (const ScheduleEntry& entry : run.schedule) {
      if (entry.arrival_tick != entry.tick) {
        throw std::runtime_error(
            "schedule arrival_tick is only supported by Unreal runs");
      }
    }
  }
  if (run.schedule.front().tick != 0 ||
      !run.schedule.front().values.complete()) {
    throw std::runtime_error("schedule tick 0 must define all four inputs");
  }
  if (run.schedule_mode == ScheduleMode::kAbsolute) {
    AircraftCommand held_input;
    for (const ScheduleEntry& entry : run.schedule) {
      entry.values.apply_to(held_input);
      validate_input(held_input, run.input_boundary);
    }
  }

  const auto& stop = run.json.at("stop");
  if (stop.contains("final_tick")) {
    run.final_tick = stop.at("final_tick");
  } else {
    const double ticks = stop.at("duration_s").get<double>() / run.dt;
    const double rounded_ticks = std::round(ticks);
    if (std::abs(ticks - rounded_ticks) > 1e-9) {
      throw std::runtime_error("stop duration is off the fixed tick grid");
    }
    run.final_tick = static_cast<std::uint64_t>(rounded_ticks);
  }
  run.output_root = run.path.parent_path() /
                    run.json.value("output_root", std::string("runs"));
  if (run.json.contains("controller")) {
    const auto& controller = run.json.at("controller");
    run.controller = ControllerConfig{
        .firmware_commit = controller.at("firmware_commit"),
        .mode = controller.at("mode"),
        .udp_port = controller.at("udp_port"),
        .startup_timeout_s = controller.at("startup_timeout_s"),
        .packet_timeout_s = controller.at("packet_timeout_s"),
        .warmup_s = controller.at("warmup_s"),
        .release_on_readiness =
            controller.value("release", std::string("timed")) == "readiness",
        .control_port =
            static_cast<std::uint16_t>(controller.value("control_port", 9003)),
        .readiness_timeout_s = controller.value("readiness_timeout_s", 45.0),
        .stable_pwm_frames = static_cast<std::uint64_t>(
            controller.value("stable_pwm_frames", 30)),
    };
    if (run.controller->release_on_readiness &&
        run.controller->control_port == run.controller->udp_port) {
      throw std::runtime_error(
          "controller control_port must differ from udp_port");
    }
    const double rate_hz = 1.0 / run.dt;
    if (std::abs(rate_hz - std::round(rate_hz)) > 1e-9) {
      throw std::runtime_error(
          "controller runs require an integer reciprocal physics rate");
    }
  }
  return run;
}

void validate_input(const AircraftCommand& input, InputBoundary boundary) {
  const double surface_limit = boundary == InputBoundary::kAircraftCommand
                                   ? 1.0
                                   : std::numbers::pi / 12.0;
  if (std::abs(input.aileron) > surface_limit ||
      std::abs(input.elevator) > surface_limit ||
      std::abs(input.rudder) > surface_limit || input.throttle < 0.0 ||
      input.throttle > 1.0) {
    throw std::runtime_error("scheduled input exceeds limits");
  }
}

AircraftEffectorState effectors_for_input(const RunConfig& run,
                                          const AircraftCommand& input) {
  validate_input(input, run.input_boundary);
  if (run.input_boundary == InputBoundary::kAircraftCommand) {
    return map_command(input, run.aircraft.parameters.actuator);
  }
  return AircraftEffectorState{
      .aileron_rad = input.aileron,
      .elevator_rad = input.elevator,
      .rudder_rad = input.rudder,
      .throttle = input.throttle,
  };
}

fs::path prepare_bundle(const RunConfig& run,
                        const std::optional<fs::path>& override_path,
                        std::string_view operation, Json& manifest) {
  const fs::path bundle =
      override_path
          ? *override_path
          : run.output_root / (run.run_id + "_" + std::string(operation) + "_" +
                               timestamp_id());
  if (fs::exists(bundle) && !fs::is_empty(bundle)) {
    throw std::runtime_error(
        "output directory already exists and is not empty: " + bundle.string());
  }
  fs::create_directories(bundle / "inputs");
  fs::create_directories(bundle / "results");

  Json resolved = run.json;
  resolved["aircraft"] = {{"path", "inputs/aircraft.json"}};
  resolved["output_root"] = ".";
  write_text(bundle / "resolved_config.json", resolved.dump(2) + "\n");
  write_text(bundle / "inputs/aircraft.json", read_text(run.aircraft.path));
  write_text(bundle / "inputs/run.json", read_text(run.path));
  write_text(bundle / "initial_state.json",
             to_json(run.initial_state).dump(2) + "\n");
  manifest = base_manifest(run, operation);
  write_text(bundle / "manifest.json", manifest.dump(2) + "\n");
  return bundle;
}

void run_validate(const fs::path& path) {
  const Json json = parse_strict(path);
  if (json.contains("model_id")) {
    const auto aircraft = load_aircraft(path);
    std::cout << "valid aircraft " << aircraft.parameters.model_id << '\n';
    return;
  }
  const auto run = load_run(path);
  std::cout << "valid run " << run.run_id << '\n';
}

}  // namespace uvd::app
