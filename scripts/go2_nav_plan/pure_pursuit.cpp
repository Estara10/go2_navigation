#include "pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace go2_nav {
namespace {

constexpr double kEps = 1e-9;

double clampDouble(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

}  // namespace

PurePursuit::PurePursuit(PurePursuitConfig config) : config_(config) {
  if (config_.lookahead_max_m < config_.lookahead_min_m) {
    std::swap(config_.lookahead_max_m, config_.lookahead_min_m);
  }
  config_.lookahead_min_m = std::max(0.0, config_.lookahead_min_m);
  config_.lookahead_max_m = std::max(config_.lookahead_min_m,
                                     config_.lookahead_max_m);
  config_.max_speed_mps = std::max(0.0, config_.max_speed_mps);
  config_.min_speed_mps =
      clampDouble(config_.min_speed_mps, 0.0, config_.max_speed_mps);
  config_.max_yaw_rate_radps = std::max(0.0, config_.max_yaw_rate_radps);
  config_.max_lateral_accel_mps2 =
      std::max(1e-6, config_.max_lateral_accel_mps2);
  config_.decel_mps2 = std::max(1e-6, config_.decel_mps2);
  config_.stop_distance_m = std::max(0.0, config_.stop_distance_m);
  config_.goal_tolerance_m = std::max(0.0, config_.goal_tolerance_m);
}

VelocityCommand PurePursuit::compute(const BodyState& state, const Path& path,
                                     PurePursuitState& tracker_state) const {
  VelocityCommand command;
  command.stamp_s = state.stamp_s;
  command.stop_request = true;

  if (!state.valid || path.empty()) {
    return command;
  }
  if (!isFinite(state.position_m.x) || !isFinite(state.position_m.y) ||
      !isFinite(state.yaw_rad) || !isFinite(state.velocity_mps.x) ||
      !isFinite(state.velocity_mps.y)) {
    return command;
  }

  const double length = path.length();
  if (length <= kEps) {
    return command;
  }

  double s_min = 0.0;
  double s_max = length;
  if (tracker_state.initialized) {
    s_min = std::max(0.0, tracker_state.last_s - 0.5);
    s_max = std::min(length, tracker_state.last_s + 2.0);
  }
  const double s_current =
      path.project(Vec2{state.position_m.x, state.position_m.y}, s_min, s_max,
                   nullptr);
  tracker_state.initialized = true;
  tracker_state.last_s = s_current;

  const double stop_window =
      std::max(config_.stop_distance_m, config_.goal_tolerance_m);
  const double remaining = length - s_current;
  if (remaining <= stop_window) {
    return command;
  }

  const double current_speed =
      std::hypot(state.velocity_mps.x, state.velocity_mps.y);
  const double nominal_speed =
      std::max(current_speed, config_.min_speed_mps);
  double lookahead =
      config_.lookahead_min_m + config_.lookahead_gain_s * nominal_speed;
  lookahead = clampDouble(lookahead, config_.lookahead_min_m,
                          config_.lookahead_max_m);
  const double s_look = std::min(s_current + lookahead, length);
  const PathPoint target = path.sample(s_look);

  const double dx = target.pose.x - state.position_m.x;
  const double dy = target.pose.y - state.position_m.y;
  const double c = std::cos(state.yaw_rad);
  const double s = std::sin(state.yaw_rad);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;

  if (local_x <= 0.0) {
    // Lookahead point is behind. Turn in place toward it. This is a fallback
    // for large initial errors; normal tracking keeps the target ahead.
    const double turn = local_y >= 0.0 ? 1.0 : -1.0;
    command.vx_mps = 0.0;
    command.vy_mps = 0.0;
    command.vyaw_radps = clampDouble(
        turn * config_.max_yaw_rate_radps, -config_.max_yaw_rate_radps,
        config_.max_yaw_rate_radps);
    command.stop_request = false;
    return command;
  }

  const double lookahead_actual = std::hypot(local_x, local_y);
  if (lookahead_actual <= kEps) {
    return command;
  }

  const double curvature =
      2.0 * local_y / (lookahead_actual * lookahead_actual);
  const double v_goal = std::sqrt(std::max(
      0.0, 2.0 * config_.decel_mps2 *
               std::max(0.0, remaining - config_.stop_distance_m)));
  const double v_curve = std::sqrt(
      config_.max_lateral_accel_mps2 /
      std::max(std::abs(curvature), 1e-6));

  double speed = std::min(config_.max_speed_mps, std::min(v_goal, v_curve));
  if (speed < config_.min_speed_mps) {
    // Keep the native gait moving until the stop window. If the curvature
    // limit is lower than the minimum speed, do not exceed that limit.
    speed = std::min(config_.min_speed_mps, v_curve);
  }
  if (speed <= kEps || remaining <= config_.stop_distance_m) {
    return command;
  }

  command.vx_mps = speed;
  command.vy_mps = 0.0;
  command.vyaw_radps =
      clampDouble(curvature * speed, -config_.max_yaw_rate_radps,
                  config_.max_yaw_rate_radps);
  command.stop_request = false;
  return command;
}

}  // namespace go2_nav
