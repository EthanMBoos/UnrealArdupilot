#include <algorithm>
#include <cmath>
#include <numbers>

#include "uvd/core.hpp"

namespace uvd {
namespace {

constexpr double kMinimumRateNormalizationAirspeedMps = 1e-12;
constexpr double kIsaMinimumAltitudeM = -1000.0;
constexpr double kIsaMaximumAltitudeM = 11000.0;

double map_surface(double command, const SurfaceMap& map) noexcept {
  const double directed_command =
      std::clamp(command, -1.0, 1.0) * map.direction;
  const double excursion_rad = directed_command >= 0.0
                                   ? map.max_rad - map.neutral_rad
                                   : map.neutral_rad - map.min_rad;
  return std::clamp(map.neutral_rad + directed_command * excursion_rad,
                    map.min_rad, map.max_rad);
}

double invert_surface(double angle_rad, const SurfaceMap& map) noexcept {
  const double delta_rad =
      std::clamp(angle_rad, map.min_rad, map.max_rad) - map.neutral_rad;
  const double excursion_rad = delta_rad >= 0.0 ? map.max_rad - map.neutral_rad
                                                : map.neutral_rad - map.min_rad;
  if (excursion_rad <= 0.0 || map.direction == 0.0) {
    return 0.0;
  }
  return std::clamp((delta_rad / excursion_rad) / map.direction, -1.0, 1.0);
}

double stable_sigmoid(double x) noexcept {
  if (x >= 0.0) {
    const double exp_negative_x = std::exp(-x);
    return 1.0 / (1.0 + exp_negative_x);
  }
  const double exp_x = std::exp(x);
  return exp_x / (1.0 + exp_x);
}

double blended_lift_coefficient(double alpha,
                                const AeroDerivatives& derivatives) noexcept {
  const double upper_stall = stable_sigmoid(
      derivatives.stall_blend_M * (alpha - derivatives.alpha_stall_rad));
  const double lower_stall = stable_sigmoid(
      -derivatives.stall_blend_M * (alpha + derivatives.alpha_stall_rad));
  const double sigma = upper_stall + lower_stall - upper_stall * lower_stall;

  const double C_L_linear = derivatives.C_L_0 + derivatives.C_L_alpha * alpha;
  const double sign_alpha = alpha > 0.0 ? 1.0 : (alpha < 0.0 ? -1.0 : 0.0);
  const double C_L_flat_plate =
      2.0 * sign_alpha * std::sin(alpha) * std::sin(alpha) * std::cos(alpha);
  return (1.0 - sigma) * C_L_linear + sigma * C_L_flat_plate;
}

}  // namespace

AtmosphereSnapshot evaluate_isa(double altitude_msl_m,
                                const Vector3& wind_ned_mps) noexcept {
  constexpr double sea_level_temperature_K = 288.15;
  constexpr double sea_level_pressure_Pa = 101325.0;
  constexpr double lapse_rate_Kpm = 0.0065;
  constexpr double gas_constant_JpkgK = 287.05287;

  const double altitude_m =
      std::clamp(altitude_msl_m, kIsaMinimumAltitudeM, kIsaMaximumAltitudeM);
  const double temperature_K =
      sea_level_temperature_K - lapse_rate_Kpm * altitude_m;
  const double pressure_Pa =
      sea_level_pressure_Pa *
      std::pow(temperature_K / sea_level_temperature_K,
               kGravityMps2 / (gas_constant_JpkgK * lapse_rate_Kpm));

  return AtmosphereSnapshot{
      .altitude_msl_m = altitude_m,
      .temperature_K = temperature_K,
      .pressure_Pa = pressure_Pa,
      .density_kgpm3 = pressure_Pa / (gas_constant_JpkgK * temperature_K),
      .wind_ned_mps = wind_ned_mps,
  };
}

AircraftEffectorState map_command(const AircraftCommand& command,
                                  const ActuatorMap& map) noexcept {
  return AircraftEffectorState{
      .aileron_rad = map_surface(command.aileron, map.aileron),
      .elevator_rad = map_surface(command.elevator, map.elevator),
      .rudder_rad = map_surface(command.rudder, map.rudder),
      .throttle = std::clamp(command.throttle, 0.0, 1.0),
  };
}

AircraftCommand invert_effector_map(const AircraftEffectorState& effectors,
                                    const ActuatorMap& map) noexcept {
  return AircraftCommand{
      .aileron = invert_surface(effectors.aileron_rad, map.aileron),
      .elevator = invert_surface(effectors.elevator_rad, map.elevator),
      .rudder = invert_surface(effectors.rudder_rad, map.rudder),
      .throttle = std::clamp(effectors.throttle, 0.0, 1.0),
  };
}

double map_pwm(std::uint16_t pwm, const PwmCalibration& calibration) noexcept {
  const double value = std::clamp(static_cast<double>(pwm),
                                  static_cast<double>(calibration.minimum),
                                  static_cast<double>(calibration.maximum));

  if (calibration.throttle) {
    const double span =
        static_cast<double>(calibration.maximum - calibration.minimum);
    const double normalized =
        span > 0.0 ? (value - calibration.minimum) / span : 0.0;
    return calibration.reversed ? 1.0 - normalized : normalized;
  }

  double normalized{};
  if (value >= calibration.trim) {
    const double span =
        static_cast<double>(calibration.maximum - calibration.trim);
    normalized = span > 0.0 ? (value - calibration.trim) / span : 0.0;
  } else {
    const double span =
        static_cast<double>(calibration.trim - calibration.minimum);
    normalized = span > 0.0 ? (value - calibration.trim) / span : 0.0;
  }
  return calibration.reversed ? -normalized : normalized;
}

AirData calculate_air_data(const RigidBodyState& state,
                           const AtmosphereSnapshot& atmosphere, double span_m,
                           double chord_m) noexcept {
  const Matrix3 R_body_to_ned =
      normalize_quaternion(state.q_body_to_ned).toRotationMatrix();
  const Vector3 wind_body_mps =
      R_body_to_ned.transpose() * atmosphere.wind_ned_mps;
  const Vector3 velocity_air_body_mps = state.velocity_body_mps - wind_body_mps;

  const double u_mps = velocity_air_body_mps.x();
  const double v_mps = velocity_air_body_mps.y();
  const double w_mps = velocity_air_body_mps.z();
  const double V_mps = velocity_air_body_mps.norm();

  AirData air{
      .velocity_air_body_mps = velocity_air_body_mps,
      .true_airspeed_mps = V_mps,
      .equivalent_airspeed_mps =
          V_mps * std::sqrt(std::max(
                      0.0, atmosphere.density_kgpm3 / kSeaLevelDensityKgpm3)),
      .alpha_rad = V_mps == 0.0 ? 0.0 : std::atan2(w_mps, u_mps),
      .beta_rad =
          V_mps == 0.0 ? 0.0 : std::atan2(v_mps, std::hypot(u_mps, w_mps)),
      .dynamic_pressure_Pa = 0.5 * atmosphere.density_kgpm3 * V_mps * V_mps,
  };

  // Below this numerical resolution, omitting normalized-rate terms avoids a
  // poorly conditioned division by V. Dynamic pressure already drives the
  // resulting aerodynamic loads smoothly to zero.
  if (V_mps > kMinimumRateNormalizationAirspeedMps) {
    const double inverse_2V = 1.0 / (2.0 * V_mps);
    air.p_hat = state.omega_body_radps.x() * span_m * inverse_2V;
    air.q_hat = state.omega_body_radps.y() * chord_m * inverse_2V;
    air.r_hat = state.omega_body_radps.z() * span_m * inverse_2V;
  }
  return air;
}

AeroCoefficientSet calculate_aero_coefficients(
    const AirData& air, const AircraftEffectorState& effectors,
    const AerosondeParameters& parameters) noexcept {
  const AeroDerivatives& d = parameters.aero;

  const double alpha = air.alpha_rad;
  const double beta = air.beta_rad;
  const double p_hat = air.p_hat;
  const double q_hat = air.q_hat;
  const double r_hat = air.r_hat;
  const double delta_a = effectors.aileron_rad;
  const double delta_e = effectors.elevator_rad;
  const double delta_r = effectors.rudder_rad;

  const double C_L_linear = d.C_L_0 + d.C_L_alpha * alpha;
  const double aspect_ratio =
      parameters.span_m * parameters.span_m / parameters.wing_area_m2;
  const double C_D_induced =
      C_L_linear * C_L_linear /
      (std::numbers::pi * parameters.oswald_efficiency * aspect_ratio);

  AeroCoefficientSet C;
  C.C_L = blended_lift_coefficient(alpha, d) + d.C_L_q * q_hat +
          d.C_L_delta_e * delta_e;
  C.C_D =
      d.C_D_parasitic + C_D_induced + d.C_D_q * q_hat + d.C_D_delta_e * delta_e;
  C.C_Y = d.C_Y_0 + d.C_Y_beta * beta + d.C_Y_p * p_hat + d.C_Y_r * r_hat +
          d.C_Y_delta_a * delta_a + d.C_Y_delta_r * delta_r;
  C.C_ell = d.C_ell_0 + d.C_ell_beta * beta + d.C_ell_p * p_hat +
            d.C_ell_r * r_hat + d.C_ell_delta_a * delta_a +
            d.C_ell_delta_r * delta_r;
  C.C_m =
      d.C_m_0 + d.C_m_alpha * alpha + d.C_m_q * q_hat + d.C_m_delta_e * delta_e;
  C.C_n = d.C_n_0 + d.C_n_beta * beta + d.C_n_p * p_hat + d.C_n_r * r_hat +
          d.C_n_delta_a * delta_a + d.C_n_delta_r * delta_r;
  return C;
}

AerodynamicsOutput evaluate_aerodynamics(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    const AtmosphereSnapshot& atmosphere,
    const AerosondeParameters& parameters) noexcept {
  AerodynamicsOutput output;
  output.air_data = calculate_air_data(state, atmosphere, parameters.span_m,
                                       parameters.chord_m);
  output.coefficients =
      calculate_aero_coefficients(output.air_data, effectors, parameters);
  if (output.air_data.true_airspeed_mps == 0.0) {
    return output;
  }

  const AeroCoefficientSet& C = output.coefficients;
  const double alpha = output.air_data.alpha_rad;
  const double cos_alpha = std::cos(alpha);
  const double sin_alpha = std::sin(alpha);
  const double qbar_S_N =
      output.air_data.dynamic_pressure_Pa * parameters.wing_area_m2;

  // FRD force and moment equations from docs/AERO.md.
  const Vector3 force_coefficient_body{
      -C.C_D * cos_alpha + C.C_L * sin_alpha,
      C.C_Y,
      -C.C_D * sin_alpha - C.C_L * cos_alpha,
  };
  const Vector3 moment_coefficient_length_body_m{
      parameters.span_m * C.C_ell,
      parameters.chord_m * C.C_m,
      parameters.span_m * C.C_n,
  };
  output.wrench.force_body_N = qbar_S_N * force_coefficient_body;
  output.wrench.moment_body_Nm = qbar_S_N * moment_coefficient_length_body_m;
  return output;
}

PropellerOutput evaluate_propeller(
    double forward_airspeed_mps, double throttle, double density_kgpm3,
    const PropellerParameters& parameters) noexcept {
  PropellerOutput output;

  const double V_a = forward_airspeed_mps;
  const double delta_t = std::clamp(throttle, 0.0, 1.0);
  const double rho = density_kgpm3;
  const double D = parameters.diameter_m;
  const double D2 = D * D;
  const double D3 = D2 * D;
  const double D4 = D3 * D;
  const double D5 = D4 * D;
  const double K_Q = parameters.motor_torque_constant_Nm_per_A;
  const double R_motor = parameters.resistance_ohm;
  const double V_in = parameters.max_voltage_V * delta_t;

  // Motor torque equals propeller torque. Substituting
  // C_Q(J) and J = 2*pi*V_a/(omega*D) gives a*omega^2+b*omega+c=0.
  const double a = rho * D5 * parameters.C_Q.x0 /
                   (4.0 * std::numbers::pi * std::numbers::pi);
  const double b =
      rho * D4 * parameters.C_Q.x1 * V_a / (2.0 * std::numbers::pi) +
      K_Q * K_Q / R_motor;
  const double c = rho * D3 * parameters.C_Q.x2 * V_a * V_a -
                   K_Q * V_in / R_motor + K_Q * parameters.no_load_current_A;

  const double discriminant = b * b - 4.0 * a * c;
  if (a == 0.0 || discriminant < 0.0 || !std::isfinite(discriminant)) {
    output.valid = false;
    return output;
  }

  output.omega_radps = (-b + std::sqrt(discriminant)) / (2.0 * a);
  if (!(output.omega_radps > 1e-9) || !std::isfinite(output.omega_radps)) {
    if (delta_t == 0.0) {
      return output;
    }
    output.valid = false;
    return output;
  }

  output.advance_ratio =
      2.0 * std::numbers::pi * V_a / (output.omega_radps * D);
  output.advance_ratio_in_range =
      output.advance_ratio >= parameters.advance_ratio_min &&
      output.advance_ratio <= parameters.advance_ratio_max;

  const double C_T = parameters.C_T.evaluate(output.advance_ratio);
  const double C_Q = parameters.C_Q.evaluate(output.advance_ratio);
  const double revolutions_per_s =
      output.omega_radps / (2.0 * std::numbers::pi);
  const double thrust_N =
      rho * revolutions_per_s * revolutions_per_s * D4 * C_T;
  const double torque_Nm =
      rho * revolutions_per_s * revolutions_per_s * D5 * C_Q;

  output.wrench.force_body_N = Vector3{thrust_N, 0.0, 0.0};
  output.wrench.moment_body_Nm =
      parameters.position_body_m.cross(output.wrench.force_body_N) +
      Vector3{-torque_Nm, 0.0, 0.0};
  output.valid =
      is_finite(output.wrench) && std::isfinite(output.advance_ratio);
  return output;
}

AircraftModelOutput evaluate_aerosonde(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    const AtmosphereSnapshot& atmosphere,
    const AerosondeParameters& parameters) noexcept {
  AircraftModelOutput output;
  output.aerodynamics =
      evaluate_aerodynamics(state, effectors, atmosphere, parameters);
  output.propulsion = evaluate_propeller(
      output.aerodynamics.air_data.velocity_air_body_mps.x(),
      effectors.throttle, atmosphere.density_kgpm3, parameters.propeller);
  output.total_wrench.force_body_N = output.aerodynamics.wrench.force_body_N +
                                     output.propulsion.wrench.force_body_N;
  output.total_wrench.moment_body_Nm =
      output.aerodynamics.wrench.moment_body_Nm +
      output.propulsion.wrench.moment_body_Nm;
  output.valid = output.propulsion.valid &&
                 is_finite(output.aerodynamics.wrench) &&
                 is_finite(output.total_wrench);
  return output;
}

}  // namespace uvd
