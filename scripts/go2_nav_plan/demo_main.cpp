#include "path.hpp"
#include "pure_pursuit.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace g = go2_nav;

namespace {

struct Result {
  bool stopped = false;
  bool all_vy_zero = true;
  double final_error_m = 0.0;
  double final_yaw_error_rad = 0.0;
  double max_yaw_rate_radps = 0.0;
  double time_s = 0.0;
};

Result run(const g::Path& path, const g::Pose2D& start,
           const g::PurePursuitConfig& config, double dt = 0.005,
           double max_time_s = 60.0) {
  Result result;
  g::PurePursuit tracker(config);
  g::PurePursuitState tracker_state;
  g::BodyState state;
  state.position_m = g::Vec3{start.x, start.y, 0.0};
  state.yaw_rad = start.yaw;
  state.valid = true;

  const int max_steps = static_cast<int>(max_time_s / dt);
  for (int step = 0; step < max_steps; ++step) {
    const g::VelocityCommand command =
        tracker.compute(state, path, tracker_state);
    result.max_yaw_rate_radps =
        std::max(result.max_yaw_rate_radps, std::abs(command.vyaw_radps));
    if (command.vy_mps != 0.0) {
      result.all_vy_zero = false;
    }
    if (command.stop_request) {
      result.stopped = true;
      result.time_s = state.stamp_s;
      break;
    }
    state.position_m.x += command.vx_mps * std::cos(state.yaw_rad) * dt;
    state.position_m.y += command.vx_mps * std::sin(state.yaw_rad) * dt;
    state.yaw_rad = g::wrapPi(state.yaw_rad + command.vyaw_radps * dt);
    state.velocity_mps = g::Vec2{command.vx_mps, 0.0};
    state.yaw_rate_radps = command.vyaw_radps;
    state.stamp_s += dt;
  }

  const g::PathPoint goal = path.points().back();
  result.final_error_m = std::hypot(state.position_m.x - goal.pose.x,
                                    state.position_m.y - goal.pose.y);
  result.final_yaw_error_rad =
      std::abs(g::wrapPi(state.yaw_rad - goal.pose.yaw));
  return result;
}

void print(const std::string& name, const g::Path& path, const Result& result) {
  std::cout << name << ": path_length=" << path.length()
            << " m, stopped=" << (result.stopped ? "yes" : "no")
            << ", final_error=" << result.final_error_m
            << " m, final_yaw_error=" << result.final_yaw_error_rad
            << " rad, time=" << result.time_s
            << " s, max_yaw_rate=" << result.max_yaw_rate_radps
            << " rad/s, vy_zero=" << (result.all_vy_zero ? "yes" : "no")
            << "\n";
}

}  // namespace

int main() {
  g::PurePursuitConfig straight_config;
  straight_config.lookahead_min_m = 0.20;
  straight_config.lookahead_gain_s = 1.0;
  straight_config.lookahead_max_m = 0.60;
  straight_config.max_speed_mps = 0.30;
  straight_config.min_speed_mps = 0.03;
  straight_config.max_yaw_rate_radps = 0.50;
  straight_config.max_lateral_accel_mps2 = 0.50;
  straight_config.decel_mps2 = 0.30;
  straight_config.stop_distance_m = 0.02;
  straight_config.goal_tolerance_m = 0.02;

  const g::Pose2D start{0.0, 0.0, 0.0};
  const g::Pose2D straight_goal{1.0, 0.0, 0.0};
  const g::Path straight = g::makeStraightPath(start, straight_goal);
  const Result straight_result = run(straight, start, straight_config);
  print("A->B straight 1.0 m", straight, straight_result);

  const g::Pose2D curve_goal{1.0, 1.0, g::kPi / 2.0};
  const g::Path curve = g::makeDubinsPath(start, curve_goal, 0.5);
  g::PurePursuitConfig curve_config = straight_config;
  curve_config.max_speed_mps = 0.20;
  const Result curve_result = run(curve, start, curve_config);
  print("A->B Dubins 1.0/1.0 m", curve, curve_result);

  const bool ok = straight_result.stopped && curve_result.stopped &&
                  straight_result.final_error_m < 0.05 &&
                  curve_result.final_error_m < 0.05 &&
                  straight_result.all_vy_zero && curve_result.all_vy_zero;
  std::cout << "simulation only; independent field measurement is still "
               "required for 5 cm acceptance\n";
  return ok ? 0 : 1;
}
