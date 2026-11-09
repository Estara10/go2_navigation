#ifndef GO2_NAV_PLAN_PATH_HPP
#define GO2_NAV_PLAN_PATH_HPP

#include <vector>

#include "body_state.hpp"

namespace go2_nav {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct PathPoint {
  Pose2D pose;
  double s = 0.0;          // arc length from the path start, m
  double curvature = 0.0;  // signed curvature, 1/m; left turn positive
};

// Dense polyline with arc-length parameterization. Points are ordered by s.
class Path {
 public:
  Path() = default;
  explicit Path(std::vector<PathPoint> points);

  bool empty() const { return points_.empty(); }
  double length() const { return length_; }
  const std::vector<PathPoint>& points() const { return points_; }

  // Clamped interpolation by arc length.
  PathPoint sample(double s) const;

  // Closest point on the polyline. Returns the arc length of the projection.
  // If distance is not null, receives the Euclidean distance to the projection.
  double project(const Vec2& point, double* distance = nullptr) const;
  double project(const Vec2& point, double s_min, double s_max,
                 double* distance = nullptr) const;

 private:
  std::vector<PathPoint> points_;
  double length_ = 0.0;
};

// Straight line from start to goal. The path yaw follows the line direction;
// it does not force goal.yaw. Use makePath() when the goal heading matters.
Path makeStraightPath(const Pose2D& start, const Pose2D& goal);

// Forward-only Dubins path with minimum turning radius radius_m. Uses the
// six-word classification (LSL, RSR, LSR, RSL, RLR, LRL) and picks the shortest.
// Returns an empty path if no solution or invalid inputs.
Path makeDubinsPath(const Pose2D& start, const Pose2D& goal, double radius_m);

// Uses a straight line when the goal is ahead and nearly on the start heading;
// otherwise uses Dubins. radius_m is the minimum turning radius.
Path makePath(const Pose2D& start, const Pose2D& goal, double radius_m);

}  // namespace go2_nav

#endif
