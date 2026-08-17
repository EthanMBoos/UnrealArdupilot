#include <cmath>

#include "uvd/core.hpp"

namespace uvd {

Quaternion normalize_quaternion(Quaternion q) noexcept {
  const double norm = q.norm();
  if (!std::isfinite(norm) || norm < 1e-15) {
    return Quaternion::Identity();
  }
  q.coeffs() /= norm;
  return q;
}

namespace {

bool quaternion_sign_is_negative(const Quaternion& q) noexcept {
  if (q.w() != 0.0) {
    return q.w() < 0.0;
  }
  if (q.x() != 0.0) {
    return q.x() < 0.0;
  }
  if (q.y() != 0.0) {
    return q.y() < 0.0;
  }
  return q.z() < 0.0;
}

}  // namespace

Quaternion canonicalize(Quaternion q) noexcept {
  q = normalize_quaternion(q);
  if (quaternion_sign_is_negative(q)) {
    q.coeffs() *= -1.0;
  }
  return q;
}

Quaternion quaternion_from_euler(double roll_rad, double pitch_rad,
                                 double yaw_rad) noexcept {
  const double cos_half_roll = std::cos(0.5 * roll_rad);
  const double sin_half_roll = std::sin(0.5 * roll_rad);
  const double cos_half_pitch = std::cos(0.5 * pitch_rad);
  const double sin_half_pitch = std::sin(0.5 * pitch_rad);
  const double cos_half_yaw = std::cos(0.5 * yaw_rad);
  const double sin_half_yaw = std::sin(0.5 * yaw_rad);

  return canonicalize(Quaternion{
      cos_half_roll * cos_half_pitch * cos_half_yaw +
          sin_half_roll * sin_half_pitch * sin_half_yaw,
      sin_half_roll * cos_half_pitch * cos_half_yaw -
          cos_half_roll * sin_half_pitch * sin_half_yaw,
      cos_half_roll * sin_half_pitch * cos_half_yaw +
          sin_half_roll * cos_half_pitch * sin_half_yaw,
      cos_half_roll * cos_half_pitch * sin_half_yaw -
          sin_half_roll * sin_half_pitch * cos_half_yaw,
  });
}

Vector3 rotation_vector(Quaternion q) noexcept {
  q = canonicalize(q);
  const Vector3 vector_part{q.x(), q.y(), q.z()};
  const double vector_norm = vector_part.norm();
  if (vector_norm < 1e-12) {
    return 2.0 * vector_part;
  }
  const double angle_rad = 2.0 * std::atan2(vector_norm, q.w());
  return (angle_rad / vector_norm) * vector_part;
}

Quaternion exp_quaternion(const Vector3& rotation_vector_rad) noexcept {
  const double angle_rad = rotation_vector_rad.norm();
  if (angle_rad < 1e-12) {
    return normalize_quaternion(Quaternion{
        1.0,
        0.5 * rotation_vector_rad.x(),
        0.5 * rotation_vector_rad.y(),
        0.5 * rotation_vector_rad.z(),
    });
  }

  const double scale = std::sin(0.5 * angle_rad) / angle_rad;
  return Quaternion{
      std::cos(0.5 * angle_rad),
      scale * rotation_vector_rad.x(),
      scale * rotation_vector_rad.y(),
      scale * rotation_vector_rad.z(),
  };
}

namespace {

Vector4 quaternion_derivative_wxyz(const Quaternion& q_body_to_ned,
                                   const Vector3& omega_body_radps) noexcept {
  const Quaternion omega_quaternion{
      0.0,
      omega_body_radps.x(),
      omega_body_radps.y(),
      omega_body_radps.z(),
  };
  const Quaternion derivative = q_body_to_ned * omega_quaternion;
  return 0.5 * Vector4{
                   derivative.w(),
                   derivative.x(),
                   derivative.y(),
                   derivative.z(),
               };
}

}  // namespace

StateDerivative rigid_body_derivative(
    const RigidBodyState& state, const BodyWrench& applied_wrench,
    double mass_kg, const Matrix3& inertia_body_kgm2) noexcept {
  const Matrix3 R_body_to_ned =
      normalize_quaternion(state.q_body_to_ned).toRotationMatrix();
  const Vector3 gravity_ned_mps2{0.0, 0.0, kGravityMps2};
  const Vector3 angular_momentum_body =
      inertia_body_kgm2 * state.omega_body_radps;

  StateDerivative derivative;
  derivative.position_dot_ned_mps = R_body_to_ned * state.velocity_body_mps;
  derivative.quaternion_dot_wxyz =
      quaternion_derivative_wxyz(state.q_body_to_ned, state.omega_body_radps);
  derivative.velocity_dot_body_mps2 =
      applied_wrench.force_body_N / mass_kg +
      R_body_to_ned.transpose() * gravity_ned_mps2 -
      state.omega_body_radps.cross(state.velocity_body_mps);
  derivative.omega_dot_body_radps2 =
      inertia_body_kgm2.inverse() *
      (applied_wrench.moment_body_Nm -
       state.omega_body_radps.cross(angular_momentum_body));
  return derivative;
}

StateDerivative evaluate_aerosonde_state_derivative(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    const AtmosphereSnapshot& atmosphere,
    const AerosondeParameters& parameters) noexcept {
  const AircraftModelOutput model =
      evaluate_aerosonde(state, effectors, atmosphere, parameters);
  return rigid_body_derivative(state, model.total_wrench, parameters.mass_kg,
                               parameters.inertia_body_kgm2);
}

bool is_finite(const RigidBodyState& state) noexcept {
  return state.position_ned_m.allFinite() &&
         state.velocity_body_mps.allFinite() &&
         state.omega_body_radps.allFinite() &&
         std::isfinite(state.q_body_to_ned.w()) &&
         std::isfinite(state.q_body_to_ned.x()) &&
         std::isfinite(state.q_body_to_ned.y()) &&
         std::isfinite(state.q_body_to_ned.z());
}

bool is_finite(const BodyWrench& wrench) noexcept {
  return wrench.force_body_N.allFinite() && wrench.moment_body_Nm.allFinite();
}

namespace {

Vector4 quaternion_wxyz(const Quaternion& quaternion) noexcept {
  return {
      quaternion.w(),
      quaternion.x(),
      quaternion.y(),
      quaternion.z(),
  };
}

Quaternion quaternion_from_wxyz(const Vector4& coefficients) noexcept {
  return Quaternion{
      coefficients(0),
      coefficients(1),
      coefficients(2),
      coefficients(3),
  };
}

RigidBodyState state_plus_scaled_derivative(const RigidBodyState& state,
                                            const StateDerivative& derivative,
                                            double scale) noexcept {
  RigidBodyState result = state;
  result.position_ned_m += scale * derivative.position_dot_ned_mps;
  result.q_body_to_ned = normalize_quaternion(
      quaternion_from_wxyz(quaternion_wxyz(result.q_body_to_ned) +
                           scale * derivative.quaternion_dot_wxyz));
  result.velocity_body_mps += scale * derivative.velocity_dot_body_mps2;
  result.omega_body_radps += scale * derivative.omega_dot_body_radps2;
  return result;
}

StateDerivative rk4_weighted_average(const StateDerivative& k1,
                                     const StateDerivative& k2,
                                     const StateDerivative& k3,
                                     const StateDerivative& k4) noexcept {
  StateDerivative average;
  average.position_dot_ned_mps =
      (k1.position_dot_ned_mps + 2.0 * k2.position_dot_ned_mps +
       2.0 * k3.position_dot_ned_mps + k4.position_dot_ned_mps) /
      6.0;
  average.quaternion_dot_wxyz =
      (k1.quaternion_dot_wxyz + 2.0 * k2.quaternion_dot_wxyz +
       2.0 * k3.quaternion_dot_wxyz + k4.quaternion_dot_wxyz) /
      6.0;
  average.velocity_dot_body_mps2 =
      (k1.velocity_dot_body_mps2 + 2.0 * k2.velocity_dot_body_mps2 +
       2.0 * k3.velocity_dot_body_mps2 + k4.velocity_dot_body_mps2) /
      6.0;
  average.omega_dot_body_radps2 =
      (k1.omega_dot_body_radps2 + 2.0 * k2.omega_dot_body_radps2 +
       2.0 * k3.omega_dot_body_radps2 + k4.omega_dot_body_radps2) /
      6.0;
  return average;
}

}  // namespace

RigidBodyState step_aerosonde_rk4(const RigidBodyState& state,
                                  const AircraftEffectorState& effectors,
                                  double origin_altitude_msl_m,
                                  const Vector3& wind_ned_mps,
                                  const AerosondeParameters& parameters,
                                  double dt_s) noexcept {
  const auto atmosphere_for = [&](const RigidBodyState& stage_state) {
    const double altitude_msl_m =
        origin_altitude_msl_m - stage_state.position_ned_m.z();
    return evaluate_isa(altitude_msl_m, wind_ned_mps);
  };

  const StateDerivative k1 = evaluate_aerosonde_state_derivative(
      state, effectors, atmosphere_for(state), parameters);
  const RigidBodyState stage2 =
      state_plus_scaled_derivative(state, k1, 0.5 * dt_s);
  const StateDerivative k2 = evaluate_aerosonde_state_derivative(
      stage2, effectors, atmosphere_for(stage2), parameters);
  const RigidBodyState stage3 =
      state_plus_scaled_derivative(state, k2, 0.5 * dt_s);
  const StateDerivative k3 = evaluate_aerosonde_state_derivative(
      stage3, effectors, atmosphere_for(stage3), parameters);
  const RigidBodyState stage4 = state_plus_scaled_derivative(state, k3, dt_s);
  const StateDerivative k4 = evaluate_aerosonde_state_derivative(
      stage4, effectors, atmosphere_for(stage4), parameters);

  RigidBodyState result = state_plus_scaled_derivative(
      state, rk4_weighted_average(k1, k2, k3, k4), dt_s);
  result.q_body_to_ned = canonicalize(result.q_body_to_ned);
  return result;
}

}  // namespace uvd
