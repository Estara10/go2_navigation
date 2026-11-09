#include "path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

namespace go2_nav {
namespace {

constexpr double kEps = 1e-12;
constexpr double kSampleStepM = 0.02;

double mod2pi(double angle) {
  double value = std::fmod(angle, 2.0 * kPi);
  if (value < 0.0) {
    value += 2.0 * kPi;
  }
  return value;
}

double clampDouble(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

struct Trig {
  double sin_a;
  double sin_b;
  double cos_a;
  double cos_b;
  double cos_ab;
};

Trig trigFuncs(double alpha, double beta) {
  return Trig{std::sin(alpha), std::sin(beta), std::cos(alpha),
              std::cos(beta), std::cos(alpha - beta)};
}

struct DubinsWord {
  bool valid = false;
  double d[3] = {0.0, 0.0, 0.0};
  char mode[3] = {'S', 'S', 'S'};
  double cost = std::numeric_limits<double>::infinity();
};

DubinsWord lsl(double alpha, double beta, double d) {
  const Trig t = trigFuncs(alpha, beta);
  const double p_squared =
      2.0 + d * d - 2.0 * t.cos_ab + 2.0 * d * (t.sin_a - t.sin_b);
  DubinsWord out;
  out.mode[0] = 'L';
  out.mode[1] = 'S';
  out.mode[2] = 'L';
  if (p_squared < 0.0) {
    return out;
  }
  const double tmp = std::atan2(t.cos_b - t.cos_a, d + t.sin_a - t.sin_b);
  out.d[0] = mod2pi(-alpha + tmp);
  out.d[1] = std::sqrt(p_squared);
  out.d[2] = mod2pi(beta - tmp);
  out.valid = true;
  out.cost = std::abs(out.d[0]) + std::abs(out.d[1]) + std::abs(out.d[2]);
  return out;
}

DubinsWord rsr(double alpha, double beta, double d) {
  const Trig t = trigFuncs(alpha, beta);
  const double p_squared =
      2.0 + d * d - 2.0 * t.cos_ab + 2.0 * d * (t.sin_b - t.sin_a);
  DubinsWord out;
  out.mode[0] = 'R';
  out.mode[1] = 'S';
  out.mode[2] = 'R';
  if (p_squared < 0.0) {
    return out;
  }
  const double tmp = std::atan2(t.cos_a - t.cos_b, d - t.sin_a + t.sin_b);
  out.d[0] = mod2pi(alpha - tmp);
  out.d[1] = std::sqrt(p_squared);
  out.d[2] = mod2pi(-beta + tmp);
  out.valid = true;
  out.cost = std::abs(out.d[0]) + std::abs(out.d[1]) + std::abs(out.d[2]);
  return out;
}

DubinsWord lsr(double alpha, double beta, double d) {
  const Trig t = trigFuncs(alpha, beta);
  const double p_squared =
      -2.0 + d * d + 2.0 * t.cos_ab + 2.0 * d * (t.sin_a + t.sin_b);
  DubinsWord out;
  out.mode[0] = 'L';
  out.mode[1] = 'S';
  out.mode[2] = 'R';
  if (p_squared < 0.0) {
    return out;
  }
  const double straight = std::sqrt(p_squared);
  const double tmp = std::atan2(-t.cos_a - t.cos_b, d + t.sin_a + t.sin_b) -
                     std::atan2(-2.0, straight);
  out.d[0] = mod2pi(-alpha + tmp);
  out.d[1] = straight;
  out.d[2] = mod2pi(-mod2pi(beta) + tmp);
  out.valid = true;
  out.cost = std::abs(out.d[0]) + std::abs(out.d[1]) + std::abs(out.d[2]);
  return out;
}

DubinsWord rsl(double alpha, double beta, double d) {
  const Trig t = trigFuncs(alpha, beta);
  const double p_squared =
      d * d - 2.0 + 2.0 * t.cos_ab - 2.0 * d * (t.sin_a + t.sin_b);
  DubinsWord out;
  out.mode[0] = 'R';
  out.mode[1] = 'S';
  out.mode[2] = 'L';
  if (p_squared < 0.0) {
    return out;
  }
  const double straight = std::sqrt(p_squared);
  const double tmp = std::atan2(t.cos_a + t.cos_b, d - t.sin_a - t.sin_b) -
                     std::atan2(2.0, straight);
  out.d[0] = mod2pi(alpha - tmp);
  out.d[1] = straight;
  out.d[2] = mod2pi(beta - tmp);
  out.valid = true;
  out.cost = std::abs(out.d[0]) + std::abs(out.d[1]) + std::abs(out.d[2]);
  return out;
}

DubinsWord rlr(double alpha, double beta, double d) {
  const Trig t = trigFuncs(alpha, beta);
  DubinsWord out;
  out.mode[0] = 'R';
  out.mode[1] = 'L';
  out.mode[2] = 'R';
  const double tmp = (6.0 - d * d + 2.0 * t.cos_ab +
                      2.0 * d * (t.sin_a - t.sin_b)) / 8.0;
  if (std::abs(tmp) > 1.0) {
    return out;
  }
  out.d[1] = mod2pi(2.0 * kPi - std::acos(tmp));
  out.d[0] = mod2pi(alpha - std::atan2(t.cos_a - t.cos_b,
                                       d - t.sin_a + t.sin_b) +
                    out.d[1] / 2.0);
  out.d[2] = mod2pi(alpha - beta - out.d[0] + out.d[1]);
  out.valid = true;
  out.cost = std::abs(out.d[0]) + std::abs(out.d[1]) + std::abs(out.d[2]);
  return out;
}

DubinsWord lrl(double alpha, double beta, double d) {
  const Trig t = trigFuncs(alpha, beta);
  DubinsWord out;
  out.mode[0] = 'L';
  out.mode[1] = 'R';
  out.mode[2] = 'L';
  const double tmp = (6.0 - d * d + 2.0 * t.cos_ab +
                      2.0 * d * (-t.sin_a + t.sin_b)) / 8.0;
  if (std::abs(tmp) > 1.0) {
    return out;
  }
  out.d[1] = mod2pi(2.0 * kPi - std::acos(tmp));
  out.d[0] = mod2pi(-alpha - std::atan2(t.cos_a - t.cos_b,
                                        d + t.sin_a - t.sin_b) +
                    out.d[1] / 2.0);
  out.d[2] = mod2pi(mod2pi(beta) - alpha - out.d[0] + mod2pi(out.d[1]));
  out.valid = true;
  out.cost = std::abs(out.d[0]) + std::abs(out.d[1]) + std::abs(out.d[2]);
  return out;
}

// Local-frame point on a Dubins segment. length_norm is the normalized segment
// length: arc angle for L/R, distance * curvature for S.
PathPoint interpolateLocal(double origin_x, double origin_y, double origin_yaw,
                           double length_norm, char mode, double curvature) {
  PathPoint point;
  if (mode == 'S') {
    const double distance = length_norm / curvature;
    point.pose.x = origin_x + distance * std::cos(origin_yaw);
    point.pose.y = origin_y + distance * std::sin(origin_yaw);
    point.pose.yaw = origin_yaw;
    point.curvature = 0.0;
    return point;
  }

  const double local_dx = std::sin(length_norm) / curvature;
  double local_dy = (1.0 - std::cos(length_norm)) / curvature;
  if (mode == 'R') {
    local_dy = -local_dy;
  }
  const double c = std::cos(-origin_yaw);
  const double s = std::sin(-origin_yaw);
  point.pose.x = origin_x + c * local_dx + s * local_dy;
  point.pose.y = origin_y - s * local_dx + c * local_dy;
  point.pose.yaw =
      origin_yaw + (mode == 'L' ? length_norm : -length_norm);
  point.curvature = (mode == 'L' ? curvature : -curvature);
  return point;
}

void appendSegment(std::vector<PathPoint>* local, const DubinsWord& word,
                   std::size_t index, double curvature, double step_m,
                   double* s_cursor) {
  const double length_norm = word.d[index];
  if (!(length_norm > kEps) || local->empty()) {
    return;
  }
  const char mode = word.mode[index];
  const PathPoint& origin = local->back();
  const double origin_x = origin.pose.x;
  const double origin_y = origin.pose.y;
  const double origin_yaw = origin.pose.yaw;
  const double step_norm = step_m * curvature;
  const double segment_length = length_norm / curvature;

  if (step_norm <= kEps) {
    return;
  }

  double current = step_norm;
  while (current < length_norm - kEps) {
    PathPoint point = interpolateLocal(origin_x, origin_y, origin_yaw, current,
                                       mode, curvature);
    point.s = *s_cursor + current / curvature;
    local->push_back(point);
    current += step_norm;
  }
  PathPoint end = interpolateLocal(origin_x, origin_y, origin_yaw, length_norm,
                                   mode, curvature);
  *s_cursor += segment_length;
  end.s = *s_cursor;
  local->push_back(end);
}

}  // namespace

Path::Path(std::vector<PathPoint> points) : points_(std::move(points)) {
  length_ = points_.empty() ? 0.0 : points_.back().s;
}

PathPoint Path::sample(double s) const {
  if (points_.empty()) {
    return PathPoint{};
  }
  if (s <= points_.front().s) {
    return points_.front();
  }
  if (s >= points_.back().s) {
    return points_.back();
  }

  std::size_t index = 0;
  while (index + 2 < points_.size() && points_[index + 1].s < s) {
    ++index;
  }
  const PathPoint& a = points_[index];
  const PathPoint& b = points_[index + 1];
  const double span = b.s - a.s;
  const double t = span > kEps ? (s - a.s) / span : 0.0;

  PathPoint out;
  out.pose.x = a.pose.x + t * (b.pose.x - a.pose.x);
  out.pose.y = a.pose.y + t * (b.pose.y - a.pose.y);
  out.pose.yaw =
      wrapPi(a.pose.yaw + t * wrapPi(b.pose.yaw - a.pose.yaw));
  out.s = s;
  out.curvature = a.curvature + t * (b.curvature - a.curvature);
  return out;
}

double Path::project(const Vec2& point, double* distance) const {
  return project(point, 0.0, length_, distance);
}

double Path::project(const Vec2& point, double s_min, double s_max,
                     double* distance) const {
  if (points_.empty()) {
    if (distance != nullptr) {
      *distance = std::numeric_limits<double>::infinity();
    }
    return 0.0;
  }
  if (points_.size() == 1) {
    const double dx = point.x - points_.front().pose.x;
    const double dy = point.y - points_.front().pose.y;
    if (distance != nullptr) {
      *distance = std::hypot(dx, dy);
    }
    return points_.front().s;
  }

  s_min = clampDouble(s_min, 0.0, length_);
  s_max = clampDouble(s_max, 0.0, length_);
  if (s_min > s_max) {
    std::swap(s_min, s_max);
  }

  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_s = s_min;
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    const PathPoint& a = points_[i];
    const PathPoint& b = points_[i + 1];
    if (b.s < s_min || a.s > s_max) {
      continue;
    }
    const double dx = b.pose.x - a.pose.x;
    const double dy = b.pose.y - a.pose.y;
    const double length_squared = dx * dx + dy * dy;
    const double span = b.s - a.s;

    double t = 0.0;
    if (length_squared > 1e-18) {
      t = ((point.x - a.pose.x) * dx + (point.y - a.pose.y) * dy) /
          length_squared;
      t = clampDouble(t, 0.0, 1.0);
    }
    double segment_s = a.s + t * span;
    if (span > kEps) {
      if (segment_s < s_min) {
        t = (s_min - a.s) / span;
        segment_s = s_min;
      } else if (segment_s > s_max) {
        t = (s_max - a.s) / span;
        segment_s = s_max;
      }
    }
    t = clampDouble(t, 0.0, 1.0);
    const double closest_x = a.pose.x + t * dx;
    const double closest_y = a.pose.y + t * dy;
    const double ddx = point.x - closest_x;
    const double ddy = point.y - closest_y;
    const double distance_squared = ddx * ddx + ddy * ddy;
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_s = segment_s;
    }
  }

  if (distance != nullptr) {
    *distance = std::sqrt(best_distance_squared);
  }
  return best_s;
}

Path makeStraightPath(const Pose2D& start, const Pose2D& goal) {
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double distance = std::hypot(dx, dy);
  const double heading = distance > 1e-9 ? std::atan2(dy, dx) : start.yaw;

  std::vector<PathPoint> points;
  if (distance <= 1e-9) {
    points.push_back(PathPoint{Pose2D{start.x, start.y, start.yaw}, 0.0, 0.0});
    return Path(std::move(points));
  }

  const std::size_t count =
      static_cast<std::size_t>(std::ceil(distance / kSampleStepM));
  points.reserve(count + 1);
  for (std::size_t i = 0; i <= count; ++i) {
    const double ratio =
        static_cast<double>(i) / static_cast<double>(count);
    const double s = distance * ratio;
    PathPoint point;
    point.pose.x = start.x + dx * ratio;
    point.pose.y = start.y + dy * ratio;
    point.pose.yaw = heading;
    point.s = s;
    point.curvature = 0.0;
    points.push_back(point);
  }
  points.back().pose.x = goal.x;
  points.back().pose.y = goal.y;
  return Path(std::move(points));
}

Path makeDubinsPath(const Pose2D& start, const Pose2D& goal,
                    double radius_m) {
  if (!(radius_m > 1e-6) || !isFinite(radius_m) || !isFinite(start.x) ||
      !isFinite(start.y) || !isFinite(start.yaw) || !isFinite(goal.x) ||
      !isFinite(goal.y) || !isFinite(goal.yaw)) {
    return Path{};
  }

  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double c = std::cos(start.yaw);
  const double s = std::sin(start.yaw);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  const double local_yaw = wrapPi(goal.yaw - start.yaw);
  const double curvature = 1.0 / radius_m;

  const double d = std::hypot(local_x, local_y) * curvature;
  const double theta = mod2pi(std::atan2(local_y, local_x));
  const double alpha = mod2pi(-theta);
  const double beta = mod2pi(local_yaw - theta);

  const std::array<DubinsWord, 6> words = {
      lsl(alpha, beta, d), rsr(alpha, beta, d), lsr(alpha, beta, d),
      rsl(alpha, beta, d), rlr(alpha, beta, d), lrl(alpha, beta, d)};

  const DubinsWord* best = nullptr;
  for (const DubinsWord& word : words) {
    if (word.valid && (best == nullptr || word.cost < best->cost)) {
      best = &word;
    }
  }
  if (best == nullptr) {
    return Path{};
  }

  std::vector<PathPoint> local;
  local.push_back(PathPoint{Pose2D{0.0, 0.0, 0.0}, 0.0, 0.0});
  double s_cursor = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    appendSegment(&local, *best, i, curvature, kSampleStepM, &s_cursor);
  }

  std::vector<PathPoint> global;
  global.reserve(local.size());
  for (const PathPoint& point : local) {
    PathPoint out = point;
    out.pose.x = start.x + c * point.pose.x - s * point.pose.y;
    out.pose.y = start.y + s * point.pose.x + c * point.pose.y;
    out.pose.yaw = wrapPi(start.yaw + point.pose.yaw);
    global.push_back(out);
  }
  return Path(std::move(global));
}

Path makePath(const Pose2D& start, const Pose2D& goal, double radius_m) {
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double c = std::cos(start.yaw);
  const double s = std::sin(start.yaw);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  const double distance = std::hypot(local_x, local_y);
  const double heading_error = std::abs(wrapPi(goal.yaw - start.yaw));

  if (distance > 1e-6 && local_x > 0.0 && std::abs(local_y) <= 0.02 &&
      heading_error <= 0.05) {
    return makeStraightPath(start, goal);
  }

  Path dubins = makeDubinsPath(start, goal, radius_m);
  if (!dubins.empty()) {
    return dubins;
  }
  return makeStraightPath(start, goal);
}

}  // namespace go2_nav
