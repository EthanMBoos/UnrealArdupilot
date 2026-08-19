#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "config.hpp"

namespace uvd::cli {
namespace {

using TrimVector = Eigen::Matrix<double, 8, 1>;
using TrimMatrix = Eigen::Matrix<double, 8, 8>;
using StateVector = Eigen::Matrix<double, 12, 1>;
using StateMatrix = Eigen::Matrix<double, 12, 12>;
using InputVector = Eigen::Matrix<double, 4, 1>;
using InputMatrix = Eigen::Matrix<double, 12, 4>;

struct TrimResult {
  RigidBodyState state;
  AircraftEffectorState effectors;
  AircraftCommand command;
  TrimVector residual = TrimVector::Zero();
  int iterations{};
  bool converged{};
  std::string message;
};

constexpr Eigen::Index kU = 0;
constexpr Eigen::Index kV = 1;
constexpr Eigen::Index kW = 2;
constexpr Eigen::Index kPitch = 3;
constexpr Eigen::Index kAileron = 4;
constexpr Eigen::Index kElevator = 5;
constexpr Eigen::Index kRudder = 6;
constexpr Eigen::Index kThrottle = 7;

RigidBodyState state_for(const TrimVector& x, const Vector3& position_ned_m) {
  return {.position_ned_m = position_ned_m,
          .q_body_to_ned = quaternion_from_euler(0.0, x(kPitch), 0.0),
          .velocity_body_mps = {x(kU), x(kV), x(kW)},
          .omega_body_radps = Vector3::Zero()};
}

AircraftEffectorState effectors_for(const TrimVector& x) {
  return {.aileron_rad = x(kAileron),
          .elevator_rad = x(kElevator),
          .rudder_rad = x(kRudder),
          .throttle = x(kThrottle)};
}

TrimVector residual_for(const TrimVector& x, const RunConfig& run,
                        double target_speed_mps) {
  const RigidBodyState state = state_for(x, run.initial_state.position_ned_m);
  const AircraftEffectorState effectors = effectors_for(x);
  const AtmosphereSnapshot atmosphere = evaluate_isa(
      run.origin_altitude_msl_m - state.position_ned_m.z(), run.wind_ned_mps);
  const StateDerivative derivative = evaluate_aerosonde_state_derivative(
      state, effectors, atmosphere, run.aircraft);
  const AirData air = calculate_air_data(state, atmosphere, run.aircraft.span_m,
                                         run.aircraft.chord_m);
  TrimVector residual;
  residual << derivative.velocity_dot_body_mps2.x(),
      derivative.velocity_dot_body_mps2.y(),
      derivative.velocity_dot_body_mps2.z(),
      derivative.omega_dot_body_radps2.x(),
      derivative.omega_dot_body_radps2.y(),
      derivative.omega_dot_body_radps2.z(),
      air.true_airspeed_mps - target_speed_mps,
      derivative.position_dot_ned_mps.z();
  return residual;
}

TrimResult solve_trim(const RunConfig& run) {
  const AircraftEffectorState initial_effectors =
      map_command(run.trim, run.aircraft.actuator);
  const double target_speed_mps = run.initial_state.velocity_body_mps.norm();
  const Matrix3 rotation = run.initial_state.q_body_to_ned.toRotationMatrix();
  TrimVector x;
  x << run.initial_state.velocity_body_mps.x(),
      run.initial_state.velocity_body_mps.y(),
      run.initial_state.velocity_body_mps.z(),
      std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
      initial_effectors.aileron_rad, initial_effectors.elevator_rad,
      initial_effectors.rudder_rad, initial_effectors.throttle;

  const double surface_limit = std::numbers::pi / 12.0;
  TrimVector lower;
  lower << 18.0, -5.0, -5.0, -0.35, -surface_limit, -surface_limit,
      -surface_limit, 0.0;
  TrimVector upper;
  upper << 40.0, 5.0, 8.0, 0.35, surface_limit, surface_limit, surface_limit,
      1.0;
  TrimVector h;
  h << 1e-4, 1e-5, 1e-5, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;

  TrimResult result;
  double damping = 1e-3;
  for (int iteration = 0; iteration < 250; ++iteration) {
    const TrimVector residual = residual_for(x, run, target_speed_mps);
    result.iterations = iteration + 1;
    if (!residual.allFinite()) {
      result.message = "nonfinite residual";
      break;
    }
    if (residual.cwiseAbs().maxCoeff() <= 1e-5) {
      result.converged = true;
      result.message = "converged";
      break;
    }

    TrimMatrix jacobian;
    for (Eigen::Index column = 0; column < x.size(); ++column) {
      TrimVector plus = x;
      TrimVector minus = x;
      plus(column) += h(column);
      minus(column) -= h(column);
      jacobian.col(column) = (residual_for(plus, run, target_speed_mps) -
                              residual_for(minus, run, target_speed_mps)) /
                             (2.0 * h(column));
    }
    const TrimVector step =
        (jacobian.transpose() * jacobian + damping * TrimMatrix::Identity())
            .ldlt()
            .solve(-jacobian.transpose() * residual);
    bool accepted = false;
    for (int line = 0; line < 16; ++line) {
      const double scale = std::ldexp(1.0, -line);
      const TrimVector candidate =
          (x + scale * step).cwiseMax(lower).cwiseMin(upper);
      const double alpha = std::atan2(candidate(kW), candidate(kU));
      const double beta =
          std::atan2(candidate(kV), std::hypot(candidate(kU), candidate(kW)));
      if (alpha < -10.0 * std::numbers::pi / 180.0 ||
          alpha > 12.0 * std::numbers::pi / 180.0 ||
          std::abs(beta) > 10.0 * std::numbers::pi / 180.0) {
        continue;
      }
      if (residual_for(candidate, run, target_speed_mps).squaredNorm() <
          residual.squaredNorm()) {
        x = candidate;
        damping = std::max(1e-12, damping * 0.3);
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      damping = std::min(1e12, damping * 10.0);
      if (damping >= 1e11) {
        result.message = "line search stalled";
        break;
      }
    }
  }
  if (result.message.empty()) {
    result.message = "iteration limit";
  }
  result.state = state_for(x, run.initial_state.position_ned_m);
  result.effectors = effectors_for(x);
  result.command = invert_effector_map(result.effectors, run.aircraft.actuator);
  result.residual = residual_for(x, run, target_speed_mps);
  return result;
}

Json trim_result_json(const TrimResult& result) {
  return {{"kind", "straight_level_trim"},
          {"state", to_json(result.state)},
          {"command", to_json(result.command)},
          {"effectors", to_json(result.effectors)},
          {"residuals",
           {{"velocity_dot_body_mps2",
             Json::array(
                 {result.residual(0), result.residual(1), result.residual(2)})},
            {"omega_dot_body_radps2",
             Json::array(
                 {result.residual(3), result.residual(4), result.residual(5)})},
            {"true_airspeed_error_mps", result.residual(6)},
            {"velocity_ned_down_mps", result.residual(7)}}},
          {"solver",
           {{"name", "bounded_damped_gauss_newton"},
            {"iterations", result.iterations},
            {"converged", result.converged},
            {"message", result.message}}}};
}

RigidBodyState apply_delta(const RigidBodyState& nominal,
                           const StateVector& delta) {
  RigidBodyState state = nominal;
  state.position_ned_m += Vector3{delta(0), delta(1), delta(2)};
  state.velocity_body_mps += Vector3{delta(3), delta(4), delta(5)};
  state.q_body_to_ned = canonicalize(
      nominal.q_body_to_ned * exp_quaternion({delta(6), delta(7), delta(8)}));
  state.omega_body_radps += Vector3{delta(9), delta(10), delta(11)};
  return state;
}

StateVector difference(const RigidBodyState& value,
                       const RigidBodyState& nominal) {
  const Vector3 position = value.position_ned_m - nominal.position_ned_m;
  const Vector3 velocity = value.velocity_body_mps - nominal.velocity_body_mps;
  const Vector3 attitude =
      rotation_vector(nominal.q_body_to_ned.conjugate() * value.q_body_to_ned);
  const Vector3 rates = value.omega_body_radps - nominal.omega_body_radps;
  StateVector result;
  result << position.x(), position.y(), position.z(), velocity.x(),
      velocity.y(), velocity.z(), attitude.x(), attitude.y(), attitude.z(),
      rates.x(), rates.y(), rates.z();
  return result;
}

Json matrix_json(const auto& matrix) {
  Json result = Json::array();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    Json values = Json::array();
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      values.push_back(matrix(row, column));
    }
    result.push_back(std::move(values));
  }
  return result;
}

}  // namespace

Json trim(const RunConfig& run) { return trim_result_json(solve_trim(run)); }

Json linearize(const RunConfig& run) {
  const TrimResult operating_point = solve_trim(run);
  if (!operating_point.converged) {
    throw std::runtime_error("trim did not converge: " +
                             operating_point.message);
  }
  const double dt_s = run.fixed_dt_s;
  const RigidBodyState nominal_next = step_aerosonde_rk4(
      operating_point.state, operating_point.effectors,
      run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft, dt_s);
  const std::array<double, 12> state_steps{1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5,
                                           1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6};
  const std::array<double, 4> input_steps{1e-5, 1e-5, 1e-5, 1e-5};
  StateMatrix A;
  InputMatrix B;

  for (Eigen::Index column = 0; column < A.cols(); ++column) {
    StateVector delta = StateVector::Zero();
    const double h = state_steps[static_cast<std::size_t>(column)];
    delta(column) = h;
    const RigidBodyState plus = step_aerosonde_rk4(
        apply_delta(operating_point.state, delta), operating_point.effectors,
        run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft, dt_s);
    const RigidBodyState minus = step_aerosonde_rk4(
        apply_delta(operating_point.state, -delta), operating_point.effectors,
        run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft, dt_s);
    A.col(column) =
        (difference(plus, nominal_next) - difference(minus, nominal_next)) /
        (2.0 * h);
  }

  InputVector command;
  command << operating_point.command.aileron, operating_point.command.elevator,
      operating_point.command.rudder, operating_point.command.throttle;
  const auto effectors_from_vector = [&](const InputVector& input) {
    return map_command({.aileron = input(0),
                        .elevator = input(1),
                        .rudder = input(2),
                        .throttle = input(3)},
                       run.aircraft.actuator);
  };
  for (Eigen::Index column = 0; column < B.cols(); ++column) {
    InputVector delta = InputVector::Zero();
    const double h = input_steps[static_cast<std::size_t>(column)];
    delta(column) = h;
    const RigidBodyState plus = step_aerosonde_rk4(
        operating_point.state, effectors_from_vector(command + delta),
        run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft, dt_s);
    const RigidBodyState minus = step_aerosonde_rk4(
        operating_point.state, effectors_from_vector(command - delta),
        run.origin_altitude_msl_m, run.wind_ned_mps, run.aircraft, dt_s);
    B.col(column) =
        (difference(plus, nominal_next) - difference(minus, nominal_next)) /
        (2.0 * h);
  }

  return {
      {"kind", "discrete_local_linear_model"},
      {"dt_s", dt_s},
      {"integrator", "rk4"},
      {"state_labels",
       Json::array({"position_north_m", "position_east_m", "position_down_m",
                    "u_mps", "v_mps", "w_mps", "attitude_error_x_rad",
                    "attitude_error_y_rad", "attitude_error_z_rad", "p_radps",
                    "q_radps", "r_radps"})},
      {"input_labels",
       Json::array({"aileron", "elevator", "rudder", "throttle"})},
      {"operating_point", trim_result_json(operating_point)},
      {"xbar_1", to_json(nominal_next)},
      {"A_d", matrix_json(A)},
      {"B_d", matrix_json(B)}};
}

}  // namespace uvd::cli
