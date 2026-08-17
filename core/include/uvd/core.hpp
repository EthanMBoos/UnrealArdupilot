#ifndef UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_CORE_HPP_
#define UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_CORE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>

namespace uvd {

inline constexpr double kGravityMps2 = 9.80665;
inline constexpr double kSeaLevelDensityKgpm3 = 1.225;

// Fixed-size Eigen objects make the mathematical dimensions explicit without
// allocating in the per-tick model.
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;

// Quaternion conventions are Hamilton (w, x, y, z), active, body to NED.
[[nodiscard]] Quaternion normalize_quaternion(Quaternion q) noexcept;
[[nodiscard]] Quaternion canonicalize(Quaternion q) noexcept;
[[nodiscard]] Quaternion quaternion_from_euler(double roll_rad,
                                               double pitch_rad,
                                               double yaw_rad) noexcept;
[[nodiscard]] Vector3 rotation_vector(Quaternion q) noexcept;
[[nodiscard]] Quaternion exp_quaternion(
    const Vector3& rotation_vector_rad) noexcept;

struct RigidBodyState {
  Vector3 position_ned_m = Vector3::Zero();
  Quaternion q_body_to_ned = Quaternion::Identity();
  Vector3 velocity_body_mps = Vector3::Zero();
  Vector3 omega_body_radps = Vector3::Zero();
};

struct BodyWrench {
  Vector3 force_body_N = Vector3::Zero();
  Vector3 moment_body_Nm = Vector3::Zero();
};

struct StepContext {
  std::uint64_t step_index{};
  double fixed_dt_s{};

  [[nodiscard]] double t0_s() const noexcept {
    return static_cast<double>(step_index) * fixed_dt_s;
  }

  [[nodiscard]] double t1_s() const noexcept {
    return static_cast<double>(step_index + 1U) * fixed_dt_s;
  }
};

struct AircraftCommand {
  double aileron{};
  double elevator{};
  double rudder{};
  double throttle{};
};

struct AircraftEffectorState {
  double aileron_rad{};
  double elevator_rad{};
  double rudder_rad{};
  double throttle{};
};

struct SurfaceMap {
  double neutral_rad{};
  double min_rad{};
  double max_rad{};
  double direction{1.0};
};

struct ActuatorMap {
  SurfaceMap aileron{};
  SurfaceMap elevator{};
  SurfaceMap rudder{};
};

struct PwmCalibration {
  std::uint16_t minimum{1000};
  std::uint16_t trim{1500};
  std::uint16_t maximum{2000};
  bool reversed{};
  bool throttle{};
};

struct AtmosphereSnapshot {
  double altitude_msl_m{};
  double temperature_K{};
  double pressure_Pa{};
  double density_kgpm3{};
  Vector3 wind_ned_mps = Vector3::Zero();
};

struct AirData {
  Vector3 velocity_air_body_mps = Vector3::Zero();
  double true_airspeed_mps{};
  double equivalent_airspeed_mps{};
  double alpha_rad{};
  double beta_rad{};
  double dynamic_pressure_Pa{};
  double p_hat{};
  double q_hat{};
  double r_hat{};
};

// Conventional nondimensional stability derivatives. C_ell denotes rolling
// moment; spelling out ell avoids confusing a lowercase l with the digit 1.
struct AeroDerivatives {
  double C_L_0{};
  double C_L_alpha{};
  double C_L_q{};
  double C_L_delta_e{};

  double C_D_parasitic{};
  double C_D_q{};
  double C_D_delta_e{};

  double C_Y_0{};
  double C_Y_beta{};
  double C_Y_p{};
  double C_Y_r{};
  double C_Y_delta_a{};
  double C_Y_delta_r{};

  double C_ell_0{};
  double C_ell_beta{};
  double C_ell_p{};
  double C_ell_r{};
  double C_ell_delta_a{};
  double C_ell_delta_r{};

  double C_m_0{};
  double C_m_alpha{};
  double C_m_q{};
  double C_m_delta_e{};

  double C_n_0{};
  double C_n_beta{};
  double C_n_p{};
  double C_n_r{};
  double C_n_delta_a{};
  double C_n_delta_r{};

  double alpha_stall_rad{0.47};
  double stall_blend_M{50.0};
};

struct AeroCoefficientSet {
  double C_L{};
  double C_D{};
  double C_Y{};
  double C_ell{};
  double C_m{};
  double C_n{};
};

// y(x) = x2*x^2 + x1*x + x0. Named terms avoid relying on an array-order
// convention in the propeller equations.
struct QuadraticPolynomial {
  double x2{};
  double x1{};
  double x0{};

  [[nodiscard]] double evaluate(double x) const noexcept {
    return x2 * x * x + x1 * x + x0;
  }
};

struct PropellerParameters {
  double diameter_m{};
  double motor_torque_constant_Nm_per_A{};
  double resistance_ohm{};
  double no_load_current_A{};
  double max_voltage_V{};
  QuadraticPolynomial C_Q{};
  QuadraticPolynomial C_T{};
  Vector3 position_body_m = Vector3::Zero();
  double advance_ratio_min{-1.0};
  double advance_ratio_max{2.0};
};

struct AerosondeParameters {
  std::string model_id{"mavsim_aerosonde_educational"};
  double mass_kg{};
  Matrix3 inertia_body_kgm2 = Matrix3::Zero();
  double wing_area_m2{};
  double span_m{};
  double chord_m{};
  double oswald_efficiency{};
  AeroDerivatives aero{};
  PropellerParameters propeller{};
  ActuatorMap actuator{};
};

struct AerodynamicsOutput {
  AirData air_data{};
  AeroCoefficientSet coefficients{};
  BodyWrench wrench{};
};

struct PropellerOutput {
  BodyWrench wrench{};
  double omega_radps{};
  double advance_ratio{};
  bool advance_ratio_in_range{true};
  bool valid{true};
};

struct AircraftModelOutput {
  AerodynamicsOutput aerodynamics{};
  PropellerOutput propulsion{};
  BodyWrench total_wrench{};
  bool valid{true};
};

struct StateDerivative {
  Vector3 position_dot_ned_mps = Vector3::Zero();
  Vector4 quaternion_dot_wxyz = Vector4::Zero();
  Vector3 velocity_dot_body_mps2 = Vector3::Zero();
  Vector3 omega_dot_body_radps2 = Vector3::Zero();
};

[[nodiscard]] AtmosphereSnapshot evaluate_isa(
    double altitude_msl_m,
    const Vector3& wind_ned_mps = Vector3::Zero()) noexcept;
[[nodiscard]] AircraftEffectorState map_command(
    const AircraftCommand& command, const ActuatorMap& map) noexcept;
[[nodiscard]] AircraftCommand invert_effector_map(
    const AircraftEffectorState& effectors, const ActuatorMap& map) noexcept;
[[nodiscard]] double map_pwm(std::uint16_t pwm,
                             const PwmCalibration& calibration) noexcept;

[[nodiscard]] AirData calculate_air_data(const RigidBodyState& state,
                                         const AtmosphereSnapshot& atmosphere,
                                         double span_m,
                                         double chord_m) noexcept;
[[nodiscard]] AeroCoefficientSet calculate_aero_coefficients(
    const AirData& air, const AircraftEffectorState& effectors,
    const AerosondeParameters& parameters) noexcept;
[[nodiscard]] AerodynamicsOutput evaluate_aerodynamics(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    const AtmosphereSnapshot& atmosphere,
    const AerosondeParameters& parameters) noexcept;
[[nodiscard]] PropellerOutput evaluate_propeller(
    double forward_airspeed_mps, double throttle, double density_kgpm3,
    const PropellerParameters& parameters) noexcept;
[[nodiscard]] AircraftModelOutput evaluate_aerosonde(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    const AtmosphereSnapshot& atmosphere,
    const AerosondeParameters& parameters) noexcept;

[[nodiscard]] StateDerivative rigid_body_derivative(
    const RigidBodyState& state, const BodyWrench& applied_wrench,
    double mass_kg, const Matrix3& inertia_body_kgm2) noexcept;
[[nodiscard]] StateDerivative evaluate_aerosonde_state_derivative(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    const AtmosphereSnapshot& atmosphere,
    const AerosondeParameters& parameters) noexcept;
[[nodiscard]] RigidBodyState step_aerosonde_rk4(
    const RigidBodyState& state, const AircraftEffectorState& effectors,
    double origin_altitude_msl_m, const Vector3& wind_ned_mps,
    const AerosondeParameters& parameters, double dt_s) noexcept;

[[nodiscard]] bool is_finite(const RigidBodyState& state) noexcept;
[[nodiscard]] bool is_finite(const BodyWrench& wrench) noexcept;

}  // namespace uvd

#endif  // UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_CORE_HPP_
