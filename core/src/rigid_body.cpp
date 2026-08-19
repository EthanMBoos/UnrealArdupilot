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
