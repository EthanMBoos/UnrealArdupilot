#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "app.hpp"

namespace fs = std::filesystem;

namespace uvd::app {
namespace {

using TrimVector = Eigen::Matrix<double, 8, 1>;
using TrimMatrix = Eigen::Matrix<double, 8, 8>;
using StateVector = Eigen::Matrix<double, 12, 1>;
using StateMatrix = Eigen::Matrix<double, 12, 12>;
using InputVector = Eigen::Matrix<double, 4, 1>;
using InputMatrix = Eigen::Matrix<double, 12, 4>;

struct TrimVariable {
  static constexpr Eigen::Index kU = 0;
  static constexpr Eigen::Index kV = 1;
  static constexpr Eigen::Index kW = 2;
  static constexpr Eigen::Index kPitch = 3;
  static constexpr Eigen::Index kAileron = 4;
  static constexpr Eigen::Index kElevator = 5;
  static constexpr Eigen::Index kRudder = 6;
  static constexpr Eigen::Index kThrottle = 7;
};

struct TrimSpecification {
  double target_true_airspeed_mps = 25.0;
  double alpha_min_rad = -10.0 * std::numbers::pi / 180.0;
  double alpha_max_rad = 12.0 * std::numbers::pi / 180.0;
  double beta_limit_rad = 10.0 * std::numbers::pi / 180.0;
  double surface_limit_rad = std::numbers::pi / 12.0;
  double residual_limit = 1e-5;
  double drift_duration_s = 10.0;
  double airspeed_drift_limit_mps = 0.05;
  double altitude_drift_limit_m = 0.5;
  double attitude_drift_limit_deg = 0.1;
};

struct TrimSolverSettings {
  int iteration_limit = 250;
  int line_search_limit = 16;
  double initial_damping = 1e-3;
  double minimum_damping = 1e-12;
  double maximum_damping = 1e12;
  double stalled_damping = 1e11;
  double accepted_damping_scale = 0.3;
  double rejected_damping_scale = 10.0;
  double bound_margin = 1e-8;
};

struct LinearizationSettings {
  std::array<double, 12> state_steps{1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5,
                                     1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6};
  std::array<double, 4> input_steps{1e-5, 1e-5, 1e-5, 1e-5};
  double convergence_limit = 1e-3;
  double doublet_amplitude = 0.01;
  double doublet_duration_s = 2.0;
  double doublet_error_limit = 0.02;
  double response_floor = 1e-9;
};

constexpr TrimSpecification kTrimSpecification;
constexpr TrimSolverSettings kTrimSolverSettings;
constexpr LinearizationSettings kLinearizationSettings;

struct TrimResult {
  RigidBodyState state{};
  AircraftEffectorState effectors{};
  AircraftCommand command{};
  TrimVector residual{};
  int iterations{};
  bool converged{};
  std::string message;
};

TrimVector initial_trim_variables() {
  TrimVector variables;
  variables(TrimVariable::kU) = 25.0;
  variables(TrimVariable::kV) = 0.0;
  variables(TrimVariable::kW) = 1.0;
  variables(TrimVariable::kPitch) = 0.04;
  variables(TrimVariable::kAileron) = 0.0;
  variables(TrimVariable::kElevator) = -0.10;
  variables(TrimVariable::kRudder) = 0.0;
  variables(TrimVariable::kThrottle) = 0.55;
  return variables;
}

TrimVector lower_trim_bounds() {
  TrimVector lower;
  lower(TrimVariable::kU) = 18.0;
  lower(TrimVariable::kV) = -5.0;
  lower(TrimVariable::kW) = -5.0;
  lower(TrimVariable::kPitch) = -0.35;
  lower(TrimVariable::kAileron) = -kTrimSpecification.surface_limit_rad;
  lower(TrimVariable::kElevator) = -kTrimSpecification.surface_limit_rad;
  lower(TrimVariable::kRudder) = -kTrimSpecification.surface_limit_rad;
  lower(TrimVariable::kThrottle) = 0.0;
  return lower;
}

TrimVector upper_trim_bounds() {
  TrimVector upper;
  upper(TrimVariable::kU) = 40.0;
  upper(TrimVariable::kV) = 5.0;
  upper(TrimVariable::kW) = 8.0;
  upper(TrimVariable::kPitch) = 0.35;
  upper(TrimVariable::kAileron) = kTrimSpecification.surface_limit_rad;
  upper(TrimVariable::kElevator) = kTrimSpecification.surface_limit_rad;
  upper(TrimVariable::kRudder) = kTrimSpecification.surface_limit_rad;
  upper(TrimVariable::kThrottle) = 1.0;
  return upper;
}

TrimVector trim_difference_steps() {
  TrimVector steps;
  steps(TrimVariable::kU) = 1e-4;
  steps(TrimVariable::kV) = 1e-5;
  steps(TrimVariable::kW) = 1e-5;
  steps(TrimVariable::kPitch) = 1e-6;
  steps(TrimVariable::kAileron) = 1e-6;
  steps(TrimVariable::kElevator) = 1e-6;
  steps(TrimVariable::kRudder) = 1e-6;
  steps(TrimVariable::kThrottle) = 1e-6;
  return steps;
}

RigidBodyState trim_state(const TrimVector& variables) {
  return RigidBodyState{
      .position_ned_m = Vector3::Zero(),
      .q_body_to_ned =
          quaternion_from_euler(0.0, variables(TrimVariable::kPitch), 0.0),
      .velocity_body_mps =
          Vector3{variables(TrimVariable::kU), variables(TrimVariable::kV),
                  variables(TrimVariable::kW)},
      .omega_body_radps = Vector3::Zero(),
  };
}

AircraftEffectorState trim_effectors(const TrimVector& variables) {
  return AircraftEffectorState{
      .aileron_rad = variables(TrimVariable::kAileron),
      .elevator_rad = variables(TrimVariable::kElevator),
      .rudder_rad = variables(TrimVariable::kRudder),
      .throttle = variables(TrimVariable::kThrottle),
  };
}

TrimVector trim_residual(const TrimVector& variables, const RunConfig& run) {
  const RigidBodyState state = trim_state(variables);
  const AircraftEffectorState effectors = trim_effectors(variables);
  const AtmosphereSnapshot atmosphere =
      evaluate_isa(run.origin_altitude_msl_m, run.wind_ned_mps);
  const StateDerivative derivative = evaluate_aerosonde_state_derivative(
      state, effectors, atmosphere, run.aircraft.parameters);
  const AirData air_data =
      calculate_air_data(state, atmosphere, run.aircraft.parameters.span_m,
                         run.aircraft.parameters.chord_m);

  TrimVector residual;
  residual << derivative.velocity_dot_body_mps2.x(),
      derivative.velocity_dot_body_mps2.y(),
      derivative.velocity_dot_body_mps2.z(),
      derivative.omega_dot_body_radps2.x(),
      derivative.omega_dot_body_radps2.y(),
      derivative.omega_dot_body_radps2.z(),
      air_data.true_airspeed_mps - kTrimSpecification.target_true_airspeed_mps,
      derivative.position_dot_ned_mps.z();
  return residual;
}

bool trim_geometry_valid(const TrimVector& variables) {
  const double alpha =
      std::atan2(variables(TrimVariable::kW), variables(TrimVariable::kU));
  const double beta = std::atan2(
      variables(TrimVariable::kV),
      std::hypot(variables(TrimVariable::kU), variables(TrimVariable::kW)));
  return alpha >= kTrimSpecification.alpha_min_rad &&
         alpha <= kTrimSpecification.alpha_max_rad &&
         std::abs(beta) <= kTrimSpecification.beta_limit_rad;
}

TrimResult solve_trim(const RunConfig& run) {
  TrimVector variables = initial_trim_variables();
  const TrimVector lower = lower_trim_bounds();
  const TrimVector upper = upper_trim_bounds();
  const TrimVector difference_steps = trim_difference_steps();
  double damping = kTrimSolverSettings.initial_damping;
  TrimResult result;

  for (int iteration = 0; iteration < kTrimSolverSettings.iteration_limit;
       ++iteration) {
    const TrimVector residual = trim_residual(variables, run);
    result.iterations = iteration + 1;
    if (!residual.allFinite()) {
      result.message = "nonfinite trim residual";
      break;
    }
    if (residual.cwiseAbs().maxCoeff() <= kTrimSpecification.residual_limit) {
      result.converged = true;
      result.message = "converged";
      break;
    }

    TrimMatrix jacobian;
    for (Eigen::Index column = 0; column < variables.size(); ++column) {
      TrimVector plus = variables;
      TrimVector minus = variables;
      plus(column) += difference_steps(column);
      minus(column) -= difference_steps(column);
      jacobian.col(column) =
          (trim_residual(plus, run) - trim_residual(minus, run)) /
          (2.0 * difference_steps(column));
    }

    const TrimMatrix normal_equations =
        jacobian.transpose() * jacobian + damping * TrimMatrix::Identity();
    const TrimVector step =
        normal_equations.ldlt().solve(-jacobian.transpose() * residual);
    if (!step.allFinite()) {
      result.message = "trim linear solve failed";
      break;
    }

    bool accepted = false;
    for (int line = 0; line < kTrimSolverSettings.line_search_limit; ++line) {
      const double scale = std::ldexp(1.0, -line);
      const TrimVector candidate =
          (variables + scale * step).cwiseMax(lower).cwiseMin(upper);
      if (!trim_geometry_valid(candidate)) {
        continue;
      }
      if (trim_residual(candidate, run).squaredNorm() <
          residual.squaredNorm()) {
        variables = candidate;
        damping =
            std::max(kTrimSolverSettings.minimum_damping,
                     damping * kTrimSolverSettings.accepted_damping_scale);
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      damping = std::min(kTrimSolverSettings.maximum_damping,
                         damping * kTrimSolverSettings.rejected_damping_scale);
      if (damping >= kTrimSolverSettings.stalled_damping) {
        result.message = "trim line search stalled";
        break;
      }
    }
  }

  result.state = trim_state(variables);
  result.effectors = trim_effectors(variables);
  result.command =
      invert_effector_map(result.effectors, run.aircraft.parameters.actuator);
  result.residual = trim_residual(variables, run);
  if (result.message.empty()) {
    result.message = "iteration limit";
  }

  const auto is_off_bound = [&](Eigen::Index index) {
    return variables(index) > lower(index) + kTrimSolverSettings.bound_margin &&
           variables(index) < upper(index) - kTrimSolverSettings.bound_margin;
  };
  result.converged = result.converged && is_off_bound(TrimVariable::kAileron) &&
                     is_off_bound(TrimVariable::kElevator) &&
                     is_off_bound(TrimVariable::kRudder) &&
                     is_off_bound(TrimVariable::kThrottle) &&
                     trim_geometry_valid(variables);
  return result;
}

Json trim_json(const TrimResult& trim, const RunConfig& run) {
  const Json residual = {
      {"velocity_dot_body_mps2",
       Json::array({trim.residual(0), trim.residual(1), trim.residual(2)})},
      {"omega_dot_body_radps2",
       Json::array({trim.residual(3), trim.residual(4), trim.residual(5)})},
      {"true_airspeed_error_mps", trim.residual(6)},
      {"velocity_ned_down_mps", trim.residual(7)}};
  return {
      {"schema_version", 1},
      {"kind", "straight_level_trim"},
      {"target_true_airspeed_mps", kTrimSpecification.target_true_airspeed_mps},
      {"state", to_json(trim.state)},
      {"command", to_json(trim.command)},
      {"effectors", to_json(trim.effectors)},
      {"residuals", residual},
      {"bounds",
       {{"alpha_deg", Json::array({-10, 12})},
        {"beta_deg", Json::array({-10, 10})},
        {"surface_deg", Json::array({-15, 15})},
        {"throttle", Json::array({0, 1})}}},
      {"solver",
       {{"name", "bounded_damped_gauss_newton"},
        {"iterations", trim.iterations},
        {"converged", trim.converged},
        {"message", trim.message}}},
      {"environment",
       {{"atmosphere", "isa"},
        {"altitude_msl_m", run.origin_altitude_msl_m},
        {"wind_ned_mps", to_json(run.wind_ned_mps)}}}};
}

RigidBodyState apply_local_state_delta(const RigidBodyState& nominal,
                                       const StateVector& delta) {
  RigidBodyState state = nominal;
  state.position_ned_m += Vector3{delta(0), delta(1), delta(2)};
  state.velocity_body_mps += Vector3{delta(3), delta(4), delta(5)};
  state.q_body_to_ned = canonicalize(
      nominal.q_body_to_ned * exp_quaternion({delta(6), delta(7), delta(8)}));
  state.omega_body_radps += Vector3{delta(9), delta(10), delta(11)};
  return state;
}

StateVector local_state_difference(const RigidBodyState& value,
                                   const RigidBodyState& nominal) {
  const Vector3 position_error = value.position_ned_m - nominal.position_ned_m;
  const Vector3 velocity_error =
      value.velocity_body_mps - nominal.velocity_body_mps;
  const Vector3 attitude_error =
      rotation_vector(nominal.q_body_to_ned.conjugate() * value.q_body_to_ned);
  const Vector3 rate_error = value.omega_body_radps - nominal.omega_body_radps;

  StateVector difference;
  difference << position_error.x(), position_error.y(), position_error.z(),
      velocity_error.x(), velocity_error.y(), velocity_error.z(),
      attitude_error.x(), attitude_error.y(), attitude_error.z(),
      rate_error.x(), rate_error.y(), rate_error.z();
  return difference;
}

InputVector nominal_input(const TrimResult& trim, InputBoundary boundary) {
  InputVector input;
  if (boundary == InputBoundary::kAircraftCommand) {
    input << trim.command.aileron, trim.command.elevator, trim.command.rudder,
        trim.command.throttle;
  } else {
    input << trim.effectors.aileron_rad, trim.effectors.elevator_rad,
        trim.effectors.rudder_rad, trim.effectors.throttle;
  }
  return input;
}

AircraftEffectorState effectors_for_linear_input(const RunConfig& run,
                                                 const InputVector& input) {
  return effectors_for_input(run, AircraftCommand{.aileron = input(0),
                                                  .elevator = input(1),
                                                  .rudder = input(2),
                                                  .throttle = input(3)});
}

std::pair<StateMatrix, InputMatrix> differentiate_step(const RunConfig& run,
                                                       const TrimResult& trim,
                                                       double scale) {
  const RigidBodyState nominal_next =
      step_aerosonde_rk4(trim.state, trim.effectors, run.origin_altitude_msl_m,
                         run.wind_ned_mps, run.aircraft.parameters, run.dt);
  StateMatrix A_d;
  InputMatrix B_d;

  for (Eigen::Index column = 0; column < A_d.cols(); ++column) {
    StateVector delta = StateVector::Zero();
    const double h =
        kLinearizationSettings.state_steps[static_cast<std::size_t>(column)] *
        scale;
    delta(column) = h;
    const RigidBodyState plus_state =
        apply_local_state_delta(trim.state, delta);
    const RigidBodyState minus_state =
        apply_local_state_delta(trim.state, -delta);
    const RigidBodyState plus = step_aerosonde_rk4(
        plus_state, trim.effectors, run.origin_altitude_msl_m, run.wind_ned_mps,
        run.aircraft.parameters, run.dt);
    const RigidBodyState minus = step_aerosonde_rk4(
        minus_state, trim.effectors, run.origin_altitude_msl_m,
        run.wind_ned_mps, run.aircraft.parameters, run.dt);
    A_d.col(column) = (local_state_difference(plus, nominal_next) -
                       local_state_difference(minus, nominal_next)) /
                      (2.0 * h);
  }

  const InputVector input_bar = nominal_input(trim, run.input_boundary);
  for (Eigen::Index column = 0; column < B_d.cols(); ++column) {
    InputVector delta = InputVector::Zero();
    const double h =
        kLinearizationSettings.input_steps[static_cast<std::size_t>(column)] *
        scale;
    delta(column) = h;
    const RigidBodyState plus = step_aerosonde_rk4(
        trim.state, effectors_for_linear_input(run, input_bar + delta),
        run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft.parameters,
        run.dt);
    const RigidBodyState minus = step_aerosonde_rk4(
        trim.state, effectors_for_linear_input(run, input_bar - delta),
        run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft.parameters,
        run.dt);
    B_d.col(column) = (local_state_difference(plus, nominal_next) -
                       local_state_difference(minus, nominal_next)) /
                      (2.0 * h);
  }
  return {A_d, B_d};
}

Json matrix_json(const Eigen::Ref<const Eigen::MatrixXd>& matrix) {
  Json result = Json::array();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    Json line = Json::array();
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      line.push_back(matrix(row, column));
    }
    result.push_back(std::move(line));
  }
  return result;
}

double relative_change(const Eigen::Ref<const Eigen::MatrixXd>& left,
                       const Eigen::Ref<const Eigen::MatrixXd>& right) {
  return (left - right).cwiseAbs().maxCoeff() /
         std::max(1.0, right.cwiseAbs().maxCoeff());
}

}  // namespace

void run_trim(const RunConfig& run, const std::optional<fs::path>& output) {
  Json manifest;
  const fs::path bundle = prepare_bundle(run, output, "trim", manifest);
  const TrimResult trim = solve_trim(run);
  Json result = trim_json(trim, run);

  RigidBodyState state = trim.state;
  const double start_true_airspeed_mps = state.velocity_body_mps.norm();
  double maximum_attitude_error_rad = 0.0;
  const auto steps = static_cast<std::uint64_t>(
      std::llround(kTrimSpecification.drift_duration_s / run.dt));
  for (std::uint64_t step = 0; step < steps; ++step) {
    state =
        step_aerosonde_rk4(state, trim.effectors, run.origin_altitude_msl_m,
                           run.wind_ned_mps, run.aircraft.parameters, run.dt);
    maximum_attitude_error_rad =
        std::max(maximum_attitude_error_rad,
                 rotation_vector(trim.state.q_body_to_ned.conjugate() *
                                 state.q_body_to_ned)
                     .norm());
  }
  const double airspeed_drift_mps =
      std::abs(state.velocity_body_mps.norm() - start_true_airspeed_mps);
  const double altitude_drift_m =
      std::abs(state.position_ned_m.z() - trim.state.position_ned_m.z());
  const double attitude_drift_deg =
      maximum_attitude_error_rad * 180.0 / std::numbers::pi;
  const bool drift_pass =
      airspeed_drift_mps < kTrimSpecification.airspeed_drift_limit_mps &&
      altitude_drift_m < kTrimSpecification.altitude_drift_limit_m &&
      attitude_drift_deg < kTrimSpecification.attitude_drift_limit_deg;

  result["drift_check"] = {{"duration_s", kTrimSpecification.drift_duration_s},
                           {"airspeed_drift_mps", airspeed_drift_mps},
                           {"altitude_drift_m", altitude_drift_m},
                           {"attitude_drift_deg", attitude_drift_deg},
                           {"passed", drift_pass}};
  result["passed"] = trim.converged && drift_pass;
  write_text(bundle / "results/operating_point.json", result.dump(2) + "\n");
  manifest["status"] = result.at("passed").get<bool>() ? "complete" : "failed";
  manifest["stop_reason"] = trim.message;
  manifest["metrics"] = {
      {"max_trim_residual", trim.residual.cwiseAbs().maxCoeff()},
      {"drift_pass", drift_pass}};
  write_text(bundle / "manifest.json", manifest.dump(2) + "\n");
  std::cout << (bundle / "results/operating_point.json") << '\n';
  if (!result.at("passed").get<bool>()) {
    throw std::runtime_error("trim acceptance checks failed");
  }
}

void run_linearize(const RunConfig& run,
                   const std::optional<fs::path>& output) {
  Json manifest;
  const fs::path bundle = prepare_bundle(run, output, "linearize", manifest);
  const TrimResult trim = solve_trim(run);
  if (!trim.converged) {
    throw std::runtime_error("cannot linearize: trim did not converge");
  }

  const auto [A_h, B_h] = differentiate_step(run, trim, 1.0);
  const auto [A_h2, B_h2] = differentiate_step(run, trim, 0.5);
  const auto [A_h4, B_h4] = differentiate_step(run, trim, 0.25);
  const double convergence_h_to_h2 =
      std::max(relative_change(A_h, A_h2), relative_change(B_h, B_h2));
  const double convergence_h2_to_h4 =
      std::max(relative_change(A_h2, A_h4), relative_change(B_h2, B_h4));

  const auto steps = static_cast<std::uint64_t>(
      std::llround(kLinearizationSettings.doublet_duration_s / run.dt));
  RigidBodyState nominal = trim.state;
  RigidBodyState nonlinear = trim.state;
  StateVector linear = StateVector::Zero();
  double maximum_error = 0.0;
  double maximum_response = kLinearizationSettings.response_floor;
  for (std::uint64_t tick = 0; tick < steps; ++tick) {
    InputVector input_delta = InputVector::Zero();
    if (tick < steps / 2) {
      input_delta(1) = kLinearizationSettings.doublet_amplitude;
    }
    nominal =
        step_aerosonde_rk4(nominal, trim.effectors, run.origin_altitude_msl_m,
                           run.wind_ned_mps, run.aircraft.parameters, run.dt);
    nonlinear = step_aerosonde_rk4(
        nonlinear,
        effectors_for_linear_input(
            run, nominal_input(trim, run.input_boundary) + input_delta),
        run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft.parameters,
        run.dt);
    linear = A_h4 * linear + B_h4 * input_delta;

    const StateVector actual = local_state_difference(nonlinear, nominal);
    const Eigen::Array<double, 12, 1> state_scales =
        (Eigen::Array<double, 12, 1>() << 1, 1, 1, 25, 25, 25, 1, 1, 1, 1, 1, 1)
            .finished();
    const double error =
        ((linear - actual).array() / state_scales).matrix().norm();
    const double response = (actual.array() / state_scales).matrix().norm();
    maximum_error = std::max(maximum_error, error);
    maximum_response = std::max(maximum_response, response);
  }
  const double doublet_error = maximum_error / maximum_response;

  const std::array<std::string, 12> state_labels{"position_north_m",
                                                 "position_east_m",
                                                 "position_down_m",
                                                 "u_mps",
                                                 "v_mps",
                                                 "w_mps",
                                                 "attitude_error_x_rad",
                                                 "attitude_error_y_rad",
                                                 "attitude_error_z_rad",
                                                 "p_radps",
                                                 "q_radps",
                                                 "r_radps"};
  const std::array<std::string, 4> command_labels{"aileron", "elevator",
                                                  "rudder", "throttle"};
  const std::array<std::string, 4> effector_labels{
      "aileron_rad", "elevator_rad", "rudder_rad", "throttle"};
  const InputVector input_bar = nominal_input(trim, run.input_boundary);
  const RigidBodyState nominal_next =
      step_aerosonde_rk4(trim.state, trim.effectors, run.origin_altitude_msl_m,
                         run.wind_ned_mps, run.aircraft.parameters, run.dt);

  Json model = {
      {"schema_version", 1},
      {"kind", "discrete_local_linear_model"},
      {"dt_s", run.dt},
      {"integrator", "rk4"},
      {"input_boundary", input_boundary_name(run.input_boundary)},
      {"state_labels", state_labels},
      {"input_labels", run.input_boundary == InputBoundary::kAircraftCommand
                           ? Json(command_labels)
                           : Json(effector_labels)},
      {"xbar_0", to_json(trim.state)},
      {"ubar",
       Json::array({input_bar(0), input_bar(1), input_bar(2), input_bar(3)})},
      {"xbar_1", to_json(nominal_next)},
      {"A_d", matrix_json(A_h4)},
      {"B_d", matrix_json(B_h4)},
      {"perturbation_scales", Json::array({"h", "h/2", "h/4"})},
      {"convergence",
       {{"h_to_h2", convergence_h_to_h2},
        {"h2_to_h4", convergence_h2_to_h4},
        {"limit", kLinearizationSettings.convergence_limit}}},
      {"doublet_check",
       {{"elevator_amplitude", kLinearizationSettings.doublet_amplitude},
        {"duration_s", kLinearizationSettings.doublet_duration_s},
        {"scaled_relative_error", doublet_error},
        {"limit", kLinearizationSettings.doublet_error_limit}}}};
  const bool pass =
      convergence_h2_to_h4 <= kLinearizationSettings.convergence_limit &&
      doublet_error <= kLinearizationSettings.doublet_error_limit;
  model["passed"] = pass;

  write_text(bundle / "results/linear_model.json", model.dump(2) + "\n");
  manifest["status"] = pass ? "complete" : "failed";
  manifest["stop_reason"] = pass ? "completed" : "acceptance_failed";
  manifest["metrics"] = {{"matrix_convergence", convergence_h2_to_h4},
                         {"doublet_error", doublet_error}};
  write_text(bundle / "manifest.json", manifest.dump(2) + "\n");
  std::cout << (bundle / "results/linear_model.json") << '\n';
  if (!pass) {
    throw std::runtime_error("linearization acceptance checks failed");
  }
}

void run_model_probe(const fs::path& aircraft_path) {
  const LoadedAircraft aircraft = load_aircraft(aircraft_path);
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }
    const Json input = Json::parse(line);
    const RigidBodyState state = state_from_json(input.at("state"));
    const auto& effector_json = input.at("effectors");
    const AircraftEffectorState effectors{
        .aileron_rad = effector_json.at("aileron_rad"),
        .elevator_rad = effector_json.at("elevator_rad"),
        .rudder_rad = effector_json.at("rudder_rad"),
        .throttle = effector_json.value("throttle", 0.0),
    };
    AtmosphereSnapshot atmosphere =
        evaluate_isa(input.value("altitude_msl_m", 0.0),
                     input.contains("wind_ned_mps")
                         ? vector3_from_json(input.at("wind_ned_mps"))
                         : Vector3::Zero());
    if (input.contains("density_kgpm3")) {
      atmosphere.density_kgpm3 = input.at("density_kgpm3");
    }

    const AerodynamicsOutput aerodynamics = evaluate_aerodynamics(
        state, effectors, atmosphere, aircraft.parameters);
    const AirData& air_data = aerodynamics.air_data;
    Json coefficients = Json::array();
    if (air_data.dynamic_pressure_Pa > 0.0) {
      const AeroCoefficientSet& C = aerodynamics.coefficients;
      coefficients = {C.C_D, C.C_L, C.C_Y, C.C_ell, C.C_m, C.C_n};
    }
    const Json output = {
        {"air_data",
         {{"tas_mps", air_data.true_airspeed_mps},
          {"alpha_rad", air_data.alpha_rad},
          {"beta_rad", air_data.beta_rad},
          {"qbar_pa", air_data.dynamic_pressure_Pa}}},
        {"coefficients", coefficients},
        {"force_n", to_json(aerodynamics.wrench.force_body_N)},
        {"moment_nm", to_json(aerodynamics.wrench.moment_body_Nm)}};
    std::cout << output.dump() << '\n';
  }
}

}  // namespace uvd::app
