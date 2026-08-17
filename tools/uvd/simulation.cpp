#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "app.hpp"

namespace fs = std::filesystem;

namespace uvd::app {
namespace {

struct TrimInput {
  AircraftCommand command;
  AircraftEffectorState effectors;
};

struct SignalSample {
  std::uint64_t tick{};
  double time_s{};
  RigidBodyState state;
  AircraftCommand input;
  AircraftEffectorState effectors;
  AtmosphereSnapshot atmosphere;
  AircraftModelOutput model;
};

class SignalCsvWriter {
 public:
  explicit SignalCsvWriter(const fs::path& path) : stream_(path) {
    if (!stream_) {
      throw std::runtime_error("cannot write signals.csv");
    }
    stream_
        << "tick,time_s,pn_m,pe_m,pd_m,qw,qx,qy,qz,u_mps,v_mps,w_mps,p_"
           "radps,q_radps,r_radps,cmd_aileron,cmd_elevator,cmd_rudder,cmd_"
           "throttle,eff_aileron_rad,eff_elevator_rad,eff_rudder_rad,eff_"
           "throttle,tas_mps,eas_mps,alpha_rad,beta_rad,rho_kgpm3,fx_aero_n,"
           "fy_aero_n,fz_aero_n,l_aero_nm,m_aero_nm,n_aero_nm,fx_prop_n,l_"
           "prop_nm,prop_j,prop_in_range\n";
    stream_ << std::setprecision(std::numeric_limits<double>::max_digits10);
  }

  void write(const SignalSample& sample) {
    const auto& state = sample.state;
    const auto& input = sample.input;
    const auto& effectors = sample.effectors;
    const auto& air_data = sample.model.aerodynamics.air_data;
    const auto& aerodynamic_wrench = sample.model.aerodynamics.wrench;
    const auto& propulsion = sample.model.propulsion;

    stream_ << sample.tick << ',' << sample.time_s << ','
            << state.position_ned_m.x() << ',' << state.position_ned_m.y()
            << ',' << state.position_ned_m.z() << ',' << state.q_body_to_ned.w()
            << ',' << state.q_body_to_ned.x() << ',' << state.q_body_to_ned.y()
            << ',' << state.q_body_to_ned.z() << ','
            << state.velocity_body_mps.x() << ',' << state.velocity_body_mps.y()
            << ',' << state.velocity_body_mps.z() << ','
            << state.omega_body_radps.x() << ',' << state.omega_body_radps.y()
            << ',' << state.omega_body_radps.z() << ',' << input.aileron << ','
            << input.elevator << ',' << input.rudder << ',' << input.throttle
            << ',' << effectors.aileron_rad << ',' << effectors.elevator_rad
            << ',' << effectors.rudder_rad << ',' << effectors.throttle << ','
            << air_data.true_airspeed_mps << ','
            << air_data.equivalent_airspeed_mps << ',' << air_data.alpha_rad
            << ',' << air_data.beta_rad << ','
            << sample.atmosphere.density_kgpm3 << ','
            << aerodynamic_wrench.force_body_N.x() << ','
            << aerodynamic_wrench.force_body_N.y() << ','
            << aerodynamic_wrench.force_body_N.z() << ','
            << aerodynamic_wrench.moment_body_Nm.x() << ','
            << aerodynamic_wrench.moment_body_Nm.y() << ','
            << aerodynamic_wrench.moment_body_Nm.z() << ','
            << propulsion.wrench.force_body_N.x() << ','
            << propulsion.wrench.moment_body_Nm.x() << ','
            << propulsion.advance_ratio << ','
            << (propulsion.advance_ratio_in_range ? 1 : 0) << '\n';
    if (!stream_) {
      throw std::runtime_error("failed writing signals.csv");
    }
  }

  void close() {
    stream_.close();
    if (!stream_) {
      throw std::runtime_error("failed closing signals.csv");
    }
  }

 private:
  std::ofstream stream_;
};

TrimInput read_trim_input(const RunConfig& run) {
  if (!run.trim_path) {
    return {};
  }
  const Json trim = parse_strict(*run.trim_path);
  const auto& command = trim.at("command");
  const auto& effectors = trim.at("effectors");
  return TrimInput{
      .command =
          AircraftCommand{
              .aileron = command.at("aileron"),
              .elevator = command.at("elevator"),
              .rudder = command.at("rudder"),
              .throttle = command.at("throttle"),
          },
      .effectors =
          AircraftEffectorState{
              .aileron_rad = effectors.at("aileron_rad"),
              .elevator_rad = effectors.at("elevator_rad"),
              .rudder_rad = effectors.at("rudder_rad"),
              .throttle = effectors.at("throttle"),
          },
  };
}

AircraftCommand add_trim_offset(const AircraftCommand& scheduled_offset,
                                const TrimInput& trim, InputBoundary boundary) {
  if (boundary == InputBoundary::kAircraftCommand) {
    return AircraftCommand{
        .aileron = trim.command.aileron + scheduled_offset.aileron,
        .elevator = trim.command.elevator + scheduled_offset.elevator,
        .rudder = trim.command.rudder + scheduled_offset.rudder,
        .throttle = trim.command.throttle + scheduled_offset.throttle,
    };
  }
  return AircraftCommand{
      .aileron = trim.effectors.aileron_rad + scheduled_offset.aileron,
      .elevator = trim.effectors.elevator_rad + scheduled_offset.elevator,
      .rudder = trim.effectors.rudder_rad + scheduled_offset.rudder,
      .throttle = trim.effectors.throttle + scheduled_offset.throttle,
  };
}

}  // namespace

SimulationResult simulate(const RunConfig& run,
                          const std::optional<fs::path>& output, bool replay) {
  Json manifest;
  const fs::path bundle =
      prepare_bundle(run, output, replay ? "replay" : "simulate", manifest);
  SignalCsvWriter signals(bundle / "signals.csv");

  const TrimInput trim = read_trim_input(run);
  AircraftCommand held_input{};
  std::size_t schedule_index = 0;
  RigidBodyState state = run.initial_state;
  bool valid = true;
  bool propeller_range_warning = false;
  std::uint64_t committed_ticks = 0;
  std::string stop_reason = "completed";

  for (std::uint64_t interval = 0; interval < run.final_tick; ++interval) {
    while (schedule_index < run.schedule.size() &&
           run.schedule[schedule_index].tick == interval) {
      run.schedule[schedule_index].values.apply_to(held_input);
      ++schedule_index;
    }

    AircraftCommand input = held_input;
    if (run.schedule_mode == ScheduleMode::kOffsetFromTrim) {
      if (!run.trim_path) {
        throw std::runtime_error("offset schedule requires controls.trim_path");
      }
      input = add_trim_offset(held_input, trim, run.input_boundary);
    }
    const AircraftEffectorState effectors = effectors_for_input(run, input);

    const AtmosphereSnapshot atmosphere = evaluate_isa(
        run.origin_altitude_msl_m - state.position_ned_m.z(), run.wind_ned_mps);
    const AircraftModelOutput model = evaluate_aerosonde(
        state, effectors, atmosphere, run.aircraft.parameters);
    const RigidBodyState next_state =
        step_aerosonde_rk4(state, effectors, run.origin_altitude_msl_m,
                           run.wind_ned_mps, run.aircraft.parameters, run.dt);
    if (!model.valid || !is_finite(next_state)) {
      valid = false;
      stop_reason = "nonfinite_or_invalid_model";
      break;
    }

    const std::uint64_t committed_tick = interval + 1;
    signals.write(SignalSample{
        .tick = committed_tick,
        .time_s = static_cast<double>(committed_tick) * run.dt,
        .state = next_state,
        .input = input,
        .effectors = effectors,
        .atmosphere = atmosphere,
        .model = model,
    });
    state = next_state;
    committed_ticks = committed_tick;
    propeller_range_warning =
        propeller_range_warning || !model.propulsion.advance_ratio_in_range;

    if (run.final_action == FinalAction::kStop &&
        schedule_index == run.schedule.size() &&
        interval >= run.schedule.back().tick) {
      stop_reason = "schedule_stop";
      break;
    }
  }

  signals.close();

  write_text(bundle / "signals.json",
             Json{{"schema_version", 1},
                  {"sample_timing",
                   "state at tick n; command, atmosphere and wrench apply over "
                   "[n-1,n)"},
                  {"frames", {{"position", "NED"}, {"body", "FRD"}}},
                  {"units", "encoded in CSV column names"}}
                     .dump(2) +
                 "\n");
  write_text(bundle / "results/final_state.json",
             to_json(state).dump(2) + "\n");
  manifest["status"] = valid ? "complete" : "failed";
  manifest["stop_reason"] = stop_reason;
  manifest["final_state_tick"] = committed_ticks;
  if (propeller_range_warning) {
    manifest["warnings"].push_back(
        "propeller advance ratio left the fitted reference range");
  }
  write_text(bundle / "manifest.json", manifest.dump(2) + "\n");
  if (!valid) {
    throw std::runtime_error("simulation failed: " + stop_reason);
  }
  return SimulationResult{
      .bundle = bundle,
      .final_state = state,
      .manifest = manifest,
  };
}

}  // namespace uvd::app
