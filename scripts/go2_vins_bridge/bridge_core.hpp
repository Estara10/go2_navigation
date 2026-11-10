#ifndef GO2_VINS_BRIDGE_BRIDGE_CORE_HPP
#define GO2_VINS_BRIDGE_BRIDGE_CORE_HPP

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "body_state.hpp"
#include "vins_wire.hpp"

namespace go2_vins_bridge {

struct BridgeConfig {
  go2_nav::Extrinsic extrinsic;
  double vins_timeout_s = 0.5;
  double jump_position_m = 0.25;
  double jump_yaw_rad = 30.0 * go2_nav::kPi / 180.0;
  double future_skew_s = 0.05;
  double header_stall_s = 0.25;
  double feature_timeout_s = 0.5;
  int minimum_feature_count = 30;
  double recovery_confirm_s = 1.0;
  double slow_drift_window_s = 3.0;
  double slow_drift_error_m = 0.25;
};

enum class HealthState {
  Uninitialized,
  Healthy,
  Failed,
  Recovering,
  RecoveryConfirmed,
};

const char* healthStateName(HealthState state);

struct VinsSample {
  double header_stamp_s = 0.0;   // VINS/ROS header time (may be Orin clock)
  double receive_stamp_s = 0.0;  // PC CLOCK_MONOTONIC at callback
  go2_nav::CameraState camera;   // camera IMU pose/twist in VINS world
};

struct Diagnostics {
  bool has_sample = false;
  bool has_nav = false;
  double header_stamp_s = 0.0;
  double receive_stamp_s = 0.0;
  go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
  HealthState health = HealthState::Uninitialized;
  int feature_count = -1;
};

class BridgeCore {
 public:
  explicit BridgeCore(BridgeConfig config);

  // Called from the ROS odometry callback. now_s is PC CLOCK_MONOTONIC.
  void onVinsSample(const VinsSample& sample, double now_s);
  // Optional feature tracker side channel. Internal to the bridge; it does
  // not alter BodyState or the frozen UDP wire format.
  void onFeatureCount(int count, double header_stamp_s, double receive_stamp_s);

  // Establishes the nav frame from the latest valid VINS base pose.
  // Returns false and a reason on failure; the caller must not start motion.
  bool setOrigin(double now_s, go2_nav::VinsReason* reason);

  // Produces the current bridge state. seq increments per call.
  go2_nav::VinsWireState makeState(double now_s);

  bool hasNav() const;
  const go2_nav::Extrinsic& extrinsic() const { return config_.extrinsic; }
  Diagnostics diagnostics() const;

 private:
  mutable std::mutex mutex_;
  BridgeConfig config_;
  bool has_sample_ = false;
  VinsSample latest_;
  double last_header_stamp_s_ = 0.0;
  double last_header_progress_receive_s_ = 0.0;
  bool header_reset_ = false;
  bool nav_established_ = false;
  bool have_state_ = false;
  go2_nav::NavFrame nav_;
  std::optional<go2_nav::BodyStateEstimator> estimator_;
  go2_nav::BodyState last_state_;
  go2_nav::VinsReason reason_ = go2_nav::VinsReason::NoVins;
  HealthState health_ = HealthState::Uninitialized;
  double recovery_start_s_ = 0.0;
  bool have_feature_sample_ = false;
  int feature_count_ = -1;
  double feature_header_stamp_s_ = 0.0;
  double last_feature_progress_receive_s_ = 0.0;
  double drift_window_start_s_ = 0.0;
  go2_nav::Vec2 drift_residual_m_{};
  bool have_drift_window_ = false;
  uint64_t seq_ = 0;
};

}  // namespace go2_vins_bridge

#endif
