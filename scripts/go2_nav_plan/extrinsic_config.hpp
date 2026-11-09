#ifndef GO2_NAV_PLAN_EXTRINSIC_CONFIG_HPP
#define GO2_NAV_PLAN_EXTRINSIC_CONFIG_HPP

#include <string>

#include "body_state.hpp"

namespace go2_nav {

// Text config shared by the offline replay tool and the ROS bridge.
// Keys: tx_m, ty_m, tz_m, yaw_deg, pitch_deg, roll_deg, camera_optical,
// measured. Values are in m and deg. measured=0 means the transform is not
// calibrated and must not be used for robot control.
struct ExtrinsicConfig {
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double roll_deg = 0.0;
  bool camera_optical = false;
  bool measured = false;
};

bool loadExtrinsicConfig(const std::string& path, ExtrinsicConfig* config,
                         std::string* error);

// Builds p_base = R_base_cam * p_cam + t_base_cam. camera_optical applies the
// nominal optical-to-base rotation (camera X right/Y down/Z forward to base
// X forward/Y left/Z up), then yaw/pitch/roll around base_link.
Extrinsic makeExtrinsic(const ExtrinsicConfig& config);

}  // namespace go2_nav

#endif
