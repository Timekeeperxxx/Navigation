#ifndef SCAN_PLANNER_REFERENCE_GUIDE_H
#define SCAN_PLANNER_REFERENCE_GUIDE_H

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <vector>

namespace scan_planner
{

struct ReferenceProgress
{
  std::size_t segment = 0;
  double ratio = 0.0;
};

inline ReferenceProgress canonicalReferenceProgress(
    std::size_t progress_segment,
    double progress_ratio,
    std::size_t segment_count)
{
  ReferenceProgress result;
  if (segment_count == 0)
    return result;

  result.segment = std::min(progress_segment, segment_count - 1);
  result.ratio = std::isfinite(progress_ratio)
                     ? std::max(0.0, std::min(1.0, progress_ratio))
                     : 0.0;
  if (result.ratio >= 1.0 - 1e-6 &&
      result.segment + 1 < segment_count)
  {
    ++result.segment;
    result.ratio = 0.0;
  }
  return result;
}

inline Eigen::Vector2d referenceForwardDirection(
    const std::vector<Eigen::Vector3d> &path,
    std::size_t progress_segment,
    double progress_ratio)
{
  if (path.size() < 2)
    return Eigen::Vector2d::Zero();

  const std::size_t segment_count = path.size() - 1;
  std::size_t segment = canonicalReferenceProgress(
      progress_segment, progress_ratio, segment_count).segment;

  for (; segment < segment_count; ++segment)
  {
    const Eigen::Vector2d direction =
        (path[segment + 1] - path[segment]).head<2>();
    if (direction.norm() > 1e-4)
      return direction;
  }
  return Eigen::Vector2d::Zero();
}

inline std::size_t findFirstReferenceStopCorner(
    const std::vector<Eigen::Vector3d> &path,
    std::size_t first_vertex,
    std::size_t last_vertex,
    const Eigen::Vector3d &start,
    double minimum_turn_angle,
    double minimum_start_distance)
{
  if (path.size() < 3)
    return path.size();

  first_vertex = std::max<std::size_t>(1, first_vertex);
  last_vertex = std::min(last_vertex, path.size() - 2);
  minimum_turn_angle = std::max(0.0, minimum_turn_angle);
  minimum_start_distance = std::max(0.0, minimum_start_distance);
  if (first_vertex > last_vertex)
    return path.size();

  for (std::size_t vertex = first_vertex;
       vertex <= last_vertex;
       ++vertex)
  {
    const Eigen::Vector2d incoming =
        (path[vertex] - path[vertex - 1]).head<2>();
    const Eigen::Vector2d outgoing =
        (path[vertex + 1] - path[vertex]).head<2>();
    if (incoming.norm() <= 1e-6 || outgoing.norm() <= 1e-6)
      continue;

    const double turn_angle = std::abs(std::atan2(
        incoming.x() * outgoing.y() - incoming.y() * outgoing.x(),
        incoming.dot(outgoing)));
    if (turn_angle + 1e-12 < minimum_turn_angle)
      continue;
    if ((path[vertex] - start).head<2>().norm() + 1e-12 <
        minimum_start_distance)
      continue;
    return vertex;
  }
  return path.size();
}

inline std::vector<Eigen::Vector3d> resampleReferenceGuide(
    const std::vector<Eigen::Vector3d> &guide,
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &target,
    double max_spacing,
    std::size_t min_points = 7)
{
  max_spacing = std::max(0.02, max_spacing);
  min_points = std::max<std::size_t>(4, min_points);

  std::vector<Eigen::Vector3d> source;
  source.reserve(guide.size() + 2);
  source.push_back(start);
  for (const auto &point : guide)
  {
    if (!point.allFinite())
      continue;
    if ((point - source.back()).norm() > 1e-5)
      source.push_back(point);
  }
  if ((target - source.back()).norm() > 1e-5)
    source.push_back(target);
  else
    source.back() = target;

  if (source.size() < 2)
    return source;

  double total_length = 0.0;
  for (std::size_t index = 1; index < source.size(); ++index)
    total_length += (source[index] - source[index - 1]).norm();
  if (total_length <= 1e-6)
    return {start, target};

  const double spacing = std::min(
      max_spacing,
      total_length / static_cast<double>(min_points - 1));

  // Retain every meaningful bend from the verified global polyline while
  // limiting the number of nearly-collinear dense samples passed to Eigen and
  // the B-spline optimizer.
  std::vector<Eigen::Vector3d> anchors;
  anchors.reserve(source.size());
  anchors.push_back(source.front());
  double distance_since_anchor = 0.0;
  constexpr double kCornerAngle = 0.035;  // about two degrees
  for (std::size_t index = 1; index + 1 < source.size(); ++index)
  {
    distance_since_anchor += (source[index] - source[index - 1]).norm();
    const Eigen::Vector2d incoming =
        (source[index] - source[index - 1]).head<2>();
    const Eigen::Vector2d outgoing =
        (source[index + 1] - source[index]).head<2>();
    double angle = 0.0;
    if (incoming.norm() > 1e-6 && outgoing.norm() > 1e-6)
    {
      const double cosine = std::max(
          -1.0,
          std::min(
              1.0,
              incoming.normalized().dot(outgoing.normalized())));
      angle = std::acos(cosine);
    }

    if (distance_since_anchor >= spacing || angle >= kCornerAngle)
    {
      anchors.push_back(source[index]);
      distance_since_anchor = 0.0;
    }
  }
  anchors.push_back(source.back());

  std::vector<Eigen::Vector3d> result;
  result.reserve(anchors.size() + min_points);
  result.push_back(anchors.front());
  for (std::size_t index = 0; index + 1 < anchors.size(); ++index)
  {
    const Eigen::Vector3d delta = anchors[index + 1] - anchors[index];
    const double length = delta.norm();
    const std::size_t divisions = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(length / spacing)));
    for (std::size_t division = 1; division <= divisions; ++division)
    {
      result.push_back(
          anchors[index] +
          (static_cast<double>(division) /
           static_cast<double>(divisions)) *
              delta);
    }
  }

  result.front() = start;
  result.back() = target;
  return result;
}

}  // namespace scan_planner

#endif
