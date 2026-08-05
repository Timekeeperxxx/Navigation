#ifndef PLAN_ENV_ROBOT_SELF_FILTER_H_
#define PLAN_ENV_ROBOT_SELF_FILTER_H_

#include <algorithm>
#include <cmath>

namespace plan_env
{

inline bool insideRobotSelfMask(
    double point_x,
    double point_y,
    double point_z,
    double body_x,
    double body_y,
    double body_yaw,
    double min_z,
    double max_z,
    double cylinder_radius,
    double cylinder_offset,
    double cylinder_center_offset,
    double padding)
{
  if (
      !std::isfinite(point_x) || !std::isfinite(point_y) || !std::isfinite(point_z) ||
      !std::isfinite(body_x) || !std::isfinite(body_y) || !std::isfinite(body_yaw) ||
      point_z < min_z || point_z > max_z)
    return false;

  const double radius = std::max(0.0, cylinder_radius) + std::max(0.0, padding);
  if (radius <= 0.0)
    return false;

  const double heading_x = std::cos(body_yaw);
  const double heading_y = std::sin(body_yaw);
  const double center_x = body_x + cylinder_center_offset * heading_x;
  const double center_y = body_y + cylinder_center_offset * heading_y;
  const double radius_sq = radius * radius;

  const auto inside_circle = [&](double circle_x, double circle_y) {
    const double dx = point_x - circle_x;
    const double dy = point_y - circle_y;
    return dx * dx + dy * dy <= radius_sq;
  };

  // Include the body origin as a centre cap.  The other two circles follow
  // the same B2 footprint geometry used by collision queries.
  return inside_circle(body_x, body_y) ||
         inside_circle(
             center_x + cylinder_offset * heading_x,
             center_y + cylinder_offset * heading_y) ||
         inside_circle(
             center_x - cylinder_offset * heading_x,
             center_y - cylinder_offset * heading_y);
}

}  // namespace plan_env

#endif  // PLAN_ENV_ROBOT_SELF_FILTER_H_
