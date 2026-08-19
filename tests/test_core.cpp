#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>

#include "uvd/core.hpp"

using Catch::Matchers::WithinAbs;

namespace {

uvd::AerosondeParameters make_parameters() {
  uvd::AerosondeParameters parameters;
  parameters.mass_kg = 11.0;
  parameters.inertia_body_kgm2 << 0.8244, 0.0, -0.1204, 0.0, 1.135, 0.0,
      -0.1204, 0.0, 1.759;
  parameters.wing_area_m2 = 0.55;
  parameters.span_m = 2.8956;
  parameters.chord_m = 0.18994;
  parameters.oswald_efficiency = 0.9;

  auto& d = parameters.aero;
  d.C_L_0 = 0.23;
  d.C_L_alpha = 5.61;
  d.C_L_q = 7.95;
  d.C_L_delta_e = 0.13;
  d.C_D_parasitic = 0.043;
  d.C_D_q = 0.0;
  d.C_D_delta_e = 0.0135;
  d.C_Y_beta = -0.98;
  d.C_Y_delta_a = 0.075;
  d.C_Y_delta_r = 0.19;
  d.C_ell_beta = -0.13;
  d.C_ell_p = -0.51;
  d.C_ell_r = 0.25;
  d.C_ell_delta_a = 0.17;
  d.C_ell_delta_r = 0.0024;
  d.C_m_0 = 0.0135;
  d.C_m_alpha = -2.74;
  d.C_m_q = -38.21;
  d.C_m_delta_e = -0.99;
  d.C_n_beta = 0.073;
  d.C_n_p = 0.069;
  d.C_n_r = -0.095;
  d.C_n_delta_a = -0.011;
  d.C_n_delta_r = -0.069;
  d.alpha_stall_rad = 0.47;
  d.stall_blend_M = 50.0;

  auto& propeller = parameters.propeller;
  propeller.diameter_m = 0.508;
  propeller.motor_torque_constant_Nm_per_A =
      (1.0 / 145.0) * 60.0 / (2.0 * std::numbers::pi);
  propeller.resistance_ohm = 0.042;
  propeller.no_load_current_A = 1.5;
  propeller.max_voltage_V = 44.4;
  propeller.C_Q = {-0.01664, 0.004970, 0.005230};
  propeller.C_T = {-0.1079, -0.06044, 0.09357};
  propeller.advance_ratio_min = 0.0;
  propeller.advance_ratio_max = 1.2;

  const uvd::SurfaceMap surface{
      .neutral_rad = 0.0,
      .min_rad = -std::numbers::pi / 12.0,
      .max_rad = std::numbers::pi / 12.0,
      .direction = 1.0,
  };
  parameters.actuator = {
      .aileron = surface,
      .elevator = surface,
      .rudder = surface,
  };
  return parameters;
}

}  // namespace

TEST_CASE("fixed-size Eigen math and quaternion conventions are consistent") {
  static_assert(uvd::Vector3::RowsAtCompileTime == 3);
  static_assert(uvd::Matrix3::RowsAtCompileTime == 3);
  static_assert(uvd::Matrix3::ColsAtCompileTime == 3);

  const uvd::Vector3 x = uvd::Vector3::UnitX();
  const uvd::Vector3 y = uvd::Vector3::UnitY();
  REQUIRE(x.cross(y).z() == 1.0);
  REQUIRE(x.dot(y) == 0.0);

  const auto q = uvd::quaternion_from_euler(0.0, 0.0, std::numbers::pi / 2.0);
  const auto rotated = q.toRotationMatrix() * x;
  REQUIRE_THAT(rotated.x(), WithinAbs(0.0, 1e-14));
  REQUIRE_THAT(rotated.y(), WithinAbs(1.0, 1e-14));
  REQUIRE_THAT(rotated.z(), WithinAbs(0.0, 1e-14));

  const auto identity = uvd::canonicalize(q * q.conjugate());
  REQUIRE_THAT(identity.w(), WithinAbs(1.0, 1e-14));

  const uvd::Vector3 rotation{0.1, -0.2, 0.3};
  const auto roundtrip = uvd::rotation_vector(uvd::exp_quaternion(rotation));
  REQUIRE_THAT((roundtrip - rotation).norm(), WithinAbs(0.0, 1e-14));
}

TEST_CASE("MSL altitude uses signed geoid undulation") {
  REQUIRE_THAT(uvd::ellipsoid_height_from_msl(100.0, -17.0),
               WithinAbs(83.0, 1e-15));
  REQUIRE_THAT(uvd::ellipsoid_height_from_msl(100.0, 31.5),
               WithinAbs(131.5, 1e-15));
}

TEST_CASE("WGS84 local NED and geodetic positions round trip") {
  const uvd::GeodeticPosition origin{
      .latitude_deg = 40.072842,
      .longitude_deg = -105.230575,
      .ellipsoid_height_m = 1584.0,
  };
  const uvd::Vector3 position_ned_m{1234.5, -678.25, -321.0};
  const auto geodetic = uvd::geodetic_from_ned(origin, position_ned_m);
  const auto round_trip = uvd::ned_from_geodetic(origin, geodetic);
  REQUIRE_THAT((round_trip - position_ned_m).norm(), WithinAbs(0.0, 1e-6));
  REQUIRE(geodetic.latitude_deg > origin.latitude_deg);
  REQUIRE(geodetic.longitude_deg < origin.longitude_deg);
  REQUIRE(geodetic.ellipsoid_height_m > origin.ellipsoid_height_m);
}

TEST_CASE("WGS84 NED cardinal directions are not swapped") {
  const uvd::GeodeticPosition origin{};
  const auto north =
      uvd::geodetic_from_ned(origin, uvd::Vector3{100.0, 0.0, 0.0});
  const auto east =
      uvd::geodetic_from_ned(origin, uvd::Vector3{0.0, 100.0, 0.0});
  const auto down =
      uvd::geodetic_from_ned(origin, uvd::Vector3{0.0, 0.0, 100.0});
  REQUIRE(north.latitude_deg > 0.0);
  REQUIRE_THAT(north.longitude_deg, WithinAbs(0.0, 1e-12));
  REQUIRE(east.longitude_deg > 0.0);
  REQUIRE_THAT(east.latitude_deg, WithinAbs(0.0, 1e-12));
  REQUIRE(down.ellipsoid_height_m < 0.0);
}

TEST_CASE("FRD inertia uses negative Jxz and inverts") {
  const auto parameters = make_parameters();
  const auto identity =
      parameters.inertia_body_kgm2 * parameters.inertia_body_kgm2.inverse();
  REQUIRE(identity.isApprox(uvd::Matrix3::Identity(), 1e-13));
}

TEST_CASE("ISA has standard sea level values and decreases density") {
  const auto sea_level = uvd::evaluate_isa(0.0);
  const auto high_altitude = uvd::evaluate_isa(3000.0);
  const auto ceiling = uvd::evaluate_isa(11000.0);
  const auto above_ceiling = uvd::evaluate_isa(12000.0);
  REQUIRE_THAT(sea_level.temperature_K, WithinAbs(288.15, 1e-12));
  REQUIRE_THAT(sea_level.pressure_Pa, WithinAbs(101325.0, 1e-9));
  REQUIRE_THAT(sea_level.density_kgpm3, WithinAbs(1.225, 2e-6));
  REQUIRE(high_altitude.density_kgpm3 < sea_level.density_kgpm3);
  REQUIRE(above_ceiling.altitude_msl_m == 11000.0);
  REQUIRE(above_ceiling.density_kgpm3 == ceiling.density_kgpm3);
}

TEST_CASE("air data honors body to NED wind conversion") {
  uvd::RigidBodyState state;
  state.velocity_body_mps = uvd::Vector3{25.0, 0.0, 0.0};
  const auto atmosphere = uvd::evaluate_isa(0.0, uvd::Vector3{5.0, 0.0, 0.0});
  const auto air = uvd::calculate_air_data(state, atmosphere, 2.0, 0.2);
  REQUIRE_THAT(air.true_airspeed_mps, WithinAbs(20.0, 1e-13));
  REQUIRE_THAT(air.alpha_rad, WithinAbs(0.0, 1e-15));
  REQUIRE_THAT(air.beta_rad, WithinAbs(0.0, 1e-15));
}

TEST_CASE(
    "normalized rate derivatives remain visible in coefficient equations") {
  const auto parameters = make_parameters();
  uvd::AirData air;
  air.alpha_rad = 0.05;
  air.beta_rad = -0.03;
  air.p_hat = 0.1;
  air.q_hat = -0.2;
  air.r_hat = 0.04;
  const uvd::AircraftEffectorState effectors{
      .aileron_rad = 0.02,
      .elevator_rad = -0.04,
      .rudder_rad = 0.03,
      .throttle = 0.5,
  };

  const auto C = uvd::calculate_aero_coefficients(air, effectors, parameters);
  const auto& d = parameters.aero;
  REQUIRE_THAT(C.C_m, WithinAbs(d.C_m_0 + d.C_m_alpha * air.alpha_rad +
                                    d.C_m_q * air.q_hat +
                                    d.C_m_delta_e * effectors.elevator_rad,
                                1e-15));
  REQUIRE_THAT(C.C_ell,
               WithinAbs(d.C_ell_beta * air.beta_rad + d.C_ell_p * air.p_hat +
                             d.C_ell_r * air.r_hat +
                             d.C_ell_delta_a * effectors.aileron_rad +
                             d.C_ell_delta_r * effectors.rudder_rad,
                         1e-15));
}

TEST_CASE("PWM mapping respects asymmetric trim and reversal") {
  const uvd::PwmCalibration surface{
      .minimum = 1100,
      .trim = 1500,
      .maximum = 1900,
  };
  REQUIRE(uvd::map_pwm(1100, surface) == -1.0);
  REQUIRE(uvd::map_pwm(1500, surface) == 0.0);
  REQUIRE(uvd::map_pwm(1900, surface) == 1.0);
  auto reversed = surface;
  reversed.reversed = true;
  REQUIRE(uvd::map_pwm(1100, reversed) == 1.0);

  const uvd::PwmCalibration throttle{
      .minimum = 1000,
      .trim = 1000,
      .maximum = 2000,
      .throttle = true,
  };
  REQUIRE(uvd::map_pwm(1500, throttle) == 0.5);
}

TEST_CASE("aerodynamics is exactly zero at zero airspeed") {
  const auto parameters = make_parameters();
  uvd::RigidBodyState state;
  state.omega_body_radps = uvd::Vector3{1.0, 2.0, 3.0};
  const auto output =
      uvd::evaluate_aerodynamics(state, {}, uvd::evaluate_isa(0.0), parameters);
  REQUIRE(output.wrench.force_body_N.isZero(0.0));
  REQUIRE(output.wrench.moment_body_Nm.isZero(0.0));

  state.velocity_body_mps = uvd::Vector3{1e-14, 0.0, 0.0};
  const auto near_zero =
      uvd::evaluate_aerodynamics(state, {}, uvd::evaluate_isa(0.0), parameters);
  REQUIRE(near_zero.air_data.p_hat == 0.0);
  REQUIRE(near_zero.air_data.q_hat == 0.0);
  REQUIRE(near_zero.air_data.r_hat == 0.0);
  REQUIRE(uvd::is_finite(near_zero.wrench));
}

TEST_CASE("aerodynamic hand signs follow FRD coefficient laws") {
  const auto parameters = make_parameters();
  uvd::RigidBodyState state;
  state.velocity_body_mps = uvd::Vector3{25.0, 0.0, 0.0};
  const auto atmosphere = uvd::evaluate_isa(0.0);

  const auto neutral =
      uvd::evaluate_aerodynamics(state, {}, atmosphere, parameters);
  REQUIRE(neutral.wrench.force_body_N.x() < 0.0);
  REQUIRE(neutral.wrench.force_body_N.z() < 0.0);

  const auto aileron = uvd::evaluate_aerodynamics(state, {.aileron_rad = 0.1},
                                                  atmosphere, parameters);
  REQUIRE(aileron.wrench.moment_body_Nm.x() >
          neutral.wrench.moment_body_Nm.x());

  const auto elevator = uvd::evaluate_aerodynamics(state, {.elevator_rad = 0.1},
                                                   atmosphere, parameters);
  REQUIRE(elevator.wrench.moment_body_Nm.y() <
          neutral.wrench.moment_body_Nm.y());

  const auto rudder = uvd::evaluate_aerodynamics(state, {.rudder_rad = 0.1},
                                                 atmosphere, parameters);
  REQUIRE(rudder.wrench.force_body_N.y() > neutral.wrench.force_body_N.y());
  REQUIRE(rudder.wrench.moment_body_Nm.z() < neutral.wrench.moment_body_Nm.z());
}

TEST_CASE("high angle and low speed sweep remains finite") {
  const auto parameters = make_parameters();
  const auto atmosphere = uvd::evaluate_isa(0.0);
  constexpr int kSpeedSteps = 30;
  constexpr int kAlphaSteps = 31;
  for (int speed_index = 0; speed_index <= kSpeedSteps; ++speed_index) {
    const double speed = 2.0 * static_cast<double>(speed_index);
    for (int alpha_index = 0; alpha_index <= kAlphaSteps; ++alpha_index) {
      const double alpha =
          -std::numbers::pi + 0.2 * static_cast<double>(alpha_index);
      uvd::RigidBodyState state;
      state.velocity_body_mps =
          uvd::Vector3{speed * std::cos(alpha), 0.0, speed * std::sin(alpha)};
      state.omega_body_radps = uvd::Vector3{0.1, -0.2, 0.1};
      const auto output = uvd::evaluate_aerodynamics(
          state, {.aileron_rad = 0.2, .elevator_rad = -0.2, .rudder_rad = 0.1},
          atmosphere, parameters);
      REQUIRE(uvd::is_finite(output.wrench));
    }
  }
}

TEST_CASE("propeller produces forward thrust and reaction torque") {
  const auto parameters = make_parameters();
  const uvd::QuadraticPolynomial polynomial{.x2 = 2.0, .x1 = 3.0, .x0 = 4.0};
  REQUIRE(polynomial.evaluate(5.0) == 69.0);

  const auto output =
      uvd::evaluate_propeller(25.0, 0.7, 1.225, parameters.propeller);
  REQUIRE(output.valid);
  REQUIRE(output.wrench.force_body_N.x() > 0.0);
  REQUIRE(output.wrench.moment_body_Nm.x() < 0.0);
  REQUIRE(std::isfinite(output.advance_ratio));
}

TEST_CASE("rigid body derivative adds gravity once") {
  uvd::BodyWrench wrench;
  wrench.force_body_N = {11.0, 0.0, 0.0};
  wrench.moment_body_Nm = {0.0, 2.0, 0.0};
  uvd::Matrix3 inertia_body_kgm2 = uvd::Matrix3::Zero();
  inertia_body_kgm2.diagonal() = uvd::Vector3{1.0, 2.0, 3.0};
  const auto derivative =
      uvd::rigid_body_derivative({}, wrench, 11.0, inertia_body_kgm2);
  REQUIRE_THAT(derivative.velocity_dot_body_mps2.x(), WithinAbs(1.0, 1e-12));
  REQUIRE_THAT(derivative.velocity_dot_body_mps2.z(),
               WithinAbs(uvd::kGravityMps2, 1e-12));
  REQUIRE_THAT(derivative.omega_dot_body_radps2.y(), WithinAbs(1.0, 1e-12));
}

TEST_CASE("RK4 commits a normalized canonical quaternion") {
  const auto parameters = make_parameters();
  uvd::RigidBodyState state;
  state.velocity_body_mps = uvd::Vector3{25.0, 0.0, 0.0};
  state.omega_body_radps = uvd::Vector3{0.1, 0.2, -0.3};
  const auto next =
      uvd::step_aerosonde_rk4(state, {.throttle = 0.5}, 0.0,
                              uvd::Vector3::Zero(), parameters, 1.0 / 120.0);
  REQUIRE_THAT(next.q_body_to_ned.norm(), WithinAbs(1.0, 1e-14));
  REQUIRE(next.q_body_to_ned.w() >= 0.0);
  REQUIRE(uvd::is_finite(next));
}
