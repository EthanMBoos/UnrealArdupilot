#include "uvd/core.hpp"

namespace uvd {
namespace {

Vector4 quaternion_wxyz(const Quaternion& q) noexcept {
  return {q.w(), q.x(), q.y(), q.z()};
}

Quaternion quaternion_from_wxyz(const Vector4& q) noexcept {
  return {q(0), q(1), q(2), q(3)};
}

RigidBodyState add(const RigidBodyState& state,
                   const StateDerivative& derivative, double scale) noexcept {
  RigidBodyState result = state;
  result.position_ned_m += scale * derivative.position_dot_ned_mps;
  result.q_body_to_ned = normalize_quaternion(
      quaternion_from_wxyz(quaternion_wxyz(state.q_body_to_ned) +
                           scale * derivative.quaternion_dot_wxyz));
  result.velocity_body_mps += scale * derivative.velocity_dot_body_mps2;
  result.omega_body_radps += scale * derivative.omega_dot_body_radps2;
  return result;
}

StateDerivative average(const StateDerivative& k1, const StateDerivative& k2,
                        const StateDerivative& k3,
                        const StateDerivative& k4) noexcept {
  StateDerivative result;
  result.position_dot_ned_mps =
      (k1.position_dot_ned_mps + 2.0 * k2.position_dot_ned_mps +
       2.0 * k3.position_dot_ned_mps + k4.position_dot_ned_mps) /
      6.0;
  result.quaternion_dot_wxyz =
      (k1.quaternion_dot_wxyz + 2.0 * k2.quaternion_dot_wxyz +
       2.0 * k3.quaternion_dot_wxyz + k4.quaternion_dot_wxyz) /
      6.0;
  result.velocity_dot_body_mps2 =
      (k1.velocity_dot_body_mps2 + 2.0 * k2.velocity_dot_body_mps2 +
       2.0 * k3.velocity_dot_body_mps2 + k4.velocity_dot_body_mps2) /
      6.0;
  result.omega_dot_body_radps2 =
      (k1.omega_dot_body_radps2 + 2.0 * k2.omega_dot_body_radps2 +
       2.0 * k3.omega_dot_body_radps2 + k4.omega_dot_body_radps2) /
      6.0;
  return result;
}

}  // namespace

StateDerivative evaluate_aerosonde_state_derivative(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    const AtmosphereSnapshot& atmosphere,
    const AerosondeParameters& parameters) noexcept {
  const AircraftModelOutput model =
      evaluate_aerosonde(state, effectors, atmosphere, parameters);
  return rigid_body_derivative(state, model.total_wrench, parameters.mass_kg,
                               parameters.inertia_body_kgm2);
}

RigidBodyState step_aerosonde_rk4(const RigidBodyState& state,
                                  const AircraftEffectorState& effectors,
                                  double origin_altitude_msl_m,
                                  const Vector3& wind_ned_mps,
                                  const AerosondeParameters& parameters,
                                  double dt_s) noexcept {
  const auto derivative = [&](const RigidBodyState& stage) {
    const double altitude_msl_m =
        origin_altitude_msl_m - stage.position_ned_m.z();
    return evaluate_aerosonde_state_derivative(
        stage, effectors, evaluate_isa(altitude_msl_m, wind_ned_mps),
        parameters);
  };

  const StateDerivative k1 = derivative(state);
  const StateDerivative k2 = derivative(add(state, k1, 0.5 * dt_s));
  const StateDerivative k3 = derivative(add(state, k2, 0.5 * dt_s));
  const StateDerivative k4 = derivative(add(state, k3, dt_s));
  RigidBodyState result = add(state, average(k1, k2, k3, k4), dt_s);
  result.q_body_to_ned = canonicalize(result.q_body_to_ned);
  return result;
}

}  // namespace uvd
