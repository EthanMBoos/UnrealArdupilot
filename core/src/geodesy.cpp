#include <algorithm>
#include <cmath>
#include <numbers>

#include "uvd/core.hpp"

namespace uvd {
namespace {

constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84InverseFlattening = 298.257223563;
constexpr double kWgs84Flattening = 1.0 / kWgs84InverseFlattening;
constexpr double kWgs84EccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kDegreesToRadians = std::numbers::pi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / std::numbers::pi;

Vector3 geodetic_to_ecef(const GeodeticPosition& position) noexcept {
  const double latitude = position.latitude_deg * kDegreesToRadians;
  const double longitude = position.longitude_deg * kDegreesToRadians;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);
  const double radius =
      kWgs84SemiMajorAxisM /
      std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
  return {
      (radius + position.ellipsoid_height_m) * cos_latitude * cos_longitude,
      (radius + position.ellipsoid_height_m) * cos_latitude * sin_longitude,
      (radius * (1.0 - kWgs84EccentricitySquared) +
       position.ellipsoid_height_m) *
          sin_latitude,
  };
}

GeodeticPosition geodetic_from_ecef(const Vector3& ecef_m) noexcept {
  const double longitude = std::atan2(ecef_m.y(), ecef_m.x());
  const double horizontal = std::hypot(ecef_m.x(), ecef_m.y());
  if (horizontal < 1e-9) {
    constexpr double kWgs84SemiMinorAxisM =
        kWgs84SemiMajorAxisM * (1.0 - kWgs84Flattening);
    return {
        .latitude_deg = std::copysign(90.0, ecef_m.z()),
        .longitude_deg = 0.0,
        .ellipsoid_height_m = std::abs(ecef_m.z()) - kWgs84SemiMinorAxisM,
    };
  }
  double latitude =
      std::atan2(ecef_m.z(), horizontal * (1.0 - kWgs84EccentricitySquared));
  double height = 0.0;
  for (int iteration = 0; iteration < 8; ++iteration) {
    const double sin_latitude = std::sin(latitude);
    const double radius =
        kWgs84SemiMajorAxisM / std::sqrt(1.0 - kWgs84EccentricitySquared *
                                                   sin_latitude * sin_latitude);
    height = horizontal / std::max(std::cos(latitude), 1e-15) - radius;
    latitude = std::atan2(
        ecef_m.z(), horizontal * (1.0 - kWgs84EccentricitySquared * radius /
                                            (radius + height)));
  }
  const double sin_latitude = std::sin(latitude);
  const double radius =
      kWgs84SemiMajorAxisM /
      std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
  height = horizontal / std::max(std::cos(latitude), 1e-15) - radius;
  return {
      .latitude_deg = latitude * kRadiansToDegrees,
      .longitude_deg = longitude * kRadiansToDegrees,
      .ellipsoid_height_m = height,
  };
}

Matrix3 ecef_to_ned_rotation(const GeodeticPosition& origin) noexcept {
  const double latitude = origin.latitude_deg * kDegreesToRadians;
  const double longitude = origin.longitude_deg * kDegreesToRadians;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);
  Matrix3 rotation;
  rotation << -sin_latitude * cos_longitude, -sin_latitude * sin_longitude,
      cos_latitude, -sin_longitude, cos_longitude, 0.0,
      -cos_latitude * cos_longitude, -cos_latitude * sin_longitude,
      -sin_latitude;
  return rotation;
}

}  // namespace

double ellipsoid_height_from_msl(double altitude_msl_m,
                                 double geoid_undulation_m) noexcept {
  return altitude_msl_m + geoid_undulation_m;
}

GeodeticPosition geodetic_from_ned(const GeodeticPosition& origin,
                                   const Vector3& position_ned_m) noexcept {
  const Vector3 ecef_m =
      geodetic_to_ecef(origin) +
      ecef_to_ned_rotation(origin).transpose() * position_ned_m;
  return geodetic_from_ecef(ecef_m);
}

Vector3 ned_from_geodetic(const GeodeticPosition& origin,
                          const GeodeticPosition& position) noexcept {
  return ecef_to_ned_rotation(origin) *
         (geodetic_to_ecef(position) - geodetic_to_ecef(origin));
}

}  // namespace uvd
