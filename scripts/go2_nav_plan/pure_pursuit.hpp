#ifndef GO2_NAV_PLAN_PURE_PURSUIT_HPP
#define GO2_NAV_PLAN_PURE_PURSUIT_HPP

#include "body_state.hpp"
#include "path.hpp"

namespace go2_nav {

struct PurePursuitConfig {
  double lookahead_min_m = 0.25;
  double lookahead_gain_s = 1.0;
  double lookahead_max_m = 0.80;
  double max_speed_mps = 0.30;
  double min_speed_mps = 0.05;
  double max_yaw_rate_radps = 0.50;
  double max_lateral_accel_mps2 = 0.50;
  double decel_mps2 = 0.30;
  double stop_distance_m = 0.03;
  double goal_tolerance_m = 0.03;
};

struct PurePursuitState {
  bool initialized = false;
  double last_s = 0.0;
};

// Standard pure-pursuit tracker with curvature- and goal-based speed
// regulation. It outputs vx/vyaw and keeps vy=0. The internal goal tolerance is
// not the 5 cm acceptance criterion; acceptance uses independent measurement.
class PurePursuit {
 public:
  explicit PurePursuit(PurePursuitConfig config = {});

  VelocityCommand compute(const BodyState& state, const Path& path,
                          PurePursuitState& tracker_state) const;

  const PurePursuitConfig& config() const { return config_; }

 private:
  PurePursuitConfig config_;
};

}  // namespace go2_nav

#endif
