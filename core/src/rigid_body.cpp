#include <cmath>

#include "uvd/core.hpp"

namespace uvd {

Quaternion normalize_quaternion(Quaternion q) noexcept {
  const double norm = q.norm();
  if (!std::isfinite(norm) || norm < 1e-12) {
    return Quaternion::Identity();
  }
  q.coeffs() /= norm;
  return q;
}

Quaternion canonicalize(Quaternion q) noexcept {
  q = normalize_quaternion(q);
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return q;
}

Quaternion quaternion_from_euler(double roll_rad, double pitch_rad,
                                 double yaw_rad) noexcept {
  const Eigen::AngleAxisd roll(roll_rad, Vector3::UnitX());
  const Eigen::AngleAxisd pitch(pitch_rad, Vector3::UnitY());
  const Eigen::AngleAxisd yaw(yaw_rad, Vector3::UnitZ());
  return canonicalize(Quaternion(yaw * pitch * roll));
}

Vector3 rotation_vector(Quaternion q) noexcept {
  q = canonicalize(q);
  const Vector3 vector_part{q.x(), q.y(), q.z()};
  const double vector_norm = vector_part.norm();
  if (vector_norm < 1e-12) {
    return 2.0 * vector_part;
  }
  return (2.0 * std::atan2(vector_norm, q.w()) / vector_norm) * vector_part;
}

Quaternion exp_quaternion(const Vector3& rotation_vector_rad) noexcept {
  const double angle_rad = rotation_vector_rad.norm();
  if (angle_rad < 1e-12) {
    return normalize_quaternion(Quaternion{1.0, 0.5 * rotation_vector_rad.x(),
                                           0.5 * rotation_vector_rad.y(),
                                           0.5 * rotation_vector_rad.z()});
  }
  const double scale = std::sin(0.5 * angle_rad) / angle_rad;
  return Quaternion{std::cos(0.5 * angle_rad), scale * rotation_vector_rad.x(),
                    scale * rotation_vector_rad.y(),
                    scale * rotation_vector_rad.z()};
}

StateDerivative rigid_body_derivative(
    const RigidBodyState& state, const BodyWrench& applied_wrench,
    double mass_kg, const Matrix3& inertia_body_kgm2) noexcept {
  const Quaternion q = normalize_quaternion(state.q_body_to_ned);
  const Matrix3 R_body_to_ned = q.toRotationMatrix();
  const Quaternion omega{0.0, state.omega_body_radps.x(),
                         state.omega_body_radps.y(),
                         state.omega_body_radps.z()};
  const Quaternion q_dot = q * omega;
  const Vector3 angular_momentum = inertia_body_kgm2 * state.omega_body_radps;

  StateDerivative derivative;
  derivative.position_dot_ned_mps = R_body_to_ned * state.velocity_body_mps;
  derivative.quaternion_dot_wxyz =
      0.5 * Vector4{q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z()};
  derivative.velocity_dot_body_mps2 =
      applied_wrench.force_body_N / mass_kg +
      R_body_to_ned.transpose() * Vector3{0.0, 0.0, kGravityMps2} -
      state.omega_body_radps.cross(state.velocity_body_mps);
  derivative.omega_dot_body_radps2 =
      inertia_body_kgm2.inverse() *
      (applied_wrench.moment_body_Nm -
       state.omega_body_radps.cross(angular_momentum));
  return derivative;
}

bool is_finite(const RigidBodyState& state) noexcept {
  return state.position_ned_m.allFinite() &&
         state.q_body_to_ned.coeffs().allFinite() &&
         state.velocity_body_mps.allFinite() &&
         state.omega_body_radps.allFinite();
}

bool is_finite(const BodyWrench& wrench) noexcept {
  return wrench.force_body_N.allFinite() && wrench.moment_body_Nm.allFinite();
}

}  // namespace uvd
