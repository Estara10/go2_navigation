#include "bridge_core.hpp"

#include <cmath>

namespace go2_vins_bridge {
namespace {

bool finiteCamera(const go2_nav::CameraState& camera) {
  return go2_nav::isFinite(camera.world_camera.position) &&
         go2_nav::isFinite(camera.world_camera.rotation) &&
         go2_nav::isFinite(camera.body_twist_camera.linear) &&
         go2_nav::isFinite(camera.body_twist_camera.angular);
}

bool finiteSample(const VinsSample& sample) {
  return sample.camera.valid_input && std::isfinite(sample.header_stamp_s) &&
         std::isfinite(sample.receive_stamp_s) &&
         finiteCamera(sample.camera);
}

bool finitePose(const go2_nav::Pose& pose) {
  return go2_nav::isFinite(pose.position) &&
         go2_nav::isFinite(pose.rotation);
}

}  // namespace

BridgeCore::BridgeCore(BridgeConfig config) : config_(config) {}

void BridgeCore::onVinsSample(const VinsSample& sample, double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(now_s) || !finiteSample(sample)) {
    reason_ = go2_nav::VinsReason::NonFinite;
    return;
  }

  if (has_sample_ &&
      sample.header_stamp_s < last_header_stamp_s_ - 1e-9) {
    header_reset_ = true;
    nav_established_ = false;
    estimator_.reset();
    have_state_ = false;
    reason_ = go2_nav::VinsReason::Reset;
  } else if (header_reset_ &&
             sample.header_stamp_s > last_header_stamp_s_ + 1e-9) {
    // A new forward header sequence arrived. The old nav frame is still
    // invalid; the native side must issue SET_ORIGIN again.
    header_reset_ = false;
  }

  latest_ = sample;
  latest_.camera.stamp_s = sample.receive_stamp_s;
  has_sample_ = true;
  last_header_stamp_s_ = sample.header_stamp_s;

  if (header_reset_) {
    reason_ = go2_nav::VinsReason::Reset;
    return;
  }
  if (!nav_established_ || !estimator_.has_value()) {
    reason_ = go2_nav::VinsReason::NoNav;
    return;
  }

  const double age_s = now_s - sample.receive_stamp_s;
  if (age_s > config_.vins_timeout_s || age_s < -config_.future_skew_s) {
    reason_ = go2_nav::VinsReason::Stale;
    return;
  }

  const go2_nav::BodyState state =
      estimator_->update(latest_.camera, now_s);
  if (!state.valid) {
    reason_ = go2_nav::VinsReason::Jump;
    return;
  }
  last_state_ = state;
  have_state_ = true;
  reason_ = go2_nav::VinsReason::Ok;
}

bool BridgeCore::setOrigin(double now_s, go2_nav::VinsReason* reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto fail = [&](go2_nav::VinsReason why) {
    reason_ = why;
    if (reason != nullptr) {
      *reason = why;
    }
    return false;
  };

  if (!std::isfinite(now_s)) {
    return fail(go2_nav::VinsReason::NonFinite);
  }
  if (!has_sample_) {
    return fail(go2_nav::VinsReason::NoVins);
  }
  if (header_reset_) {
    return fail(go2_nav::VinsReason::Reset);
  }
  if (!finiteSample(latest_)) {
    return fail(go2_nav::VinsReason::NonFinite);
  }
  const double age_s = now_s - latest_.receive_stamp_s;
  if (age_s > config_.vins_timeout_s || age_s < -config_.future_skew_s) {
    return fail(go2_nav::VinsReason::Stale);
  }

  const go2_nav::Pose base_pose =
      go2_nav::basePoseFromCamera(latest_.camera.world_camera,
                                   config_.extrinsic);
  if (!finitePose(base_pose)) {
    return fail(go2_nav::VinsReason::NonFinite);
  }

  nav_ = go2_nav::NavFrame::fromInitialBasePose(base_pose);
  go2_nav::ValidityConfig validity;
  validity.max_age_s = config_.vins_timeout_s;
  validity.max_future_skew_s = config_.future_skew_s;
  validity.max_position_jump_m = config_.jump_position_m;
  validity.max_yaw_jump_rad = config_.jump_yaw_rad;
  estimator_.emplace(nav_, config_.extrinsic, validity);

  nav_established_ = true;
  header_reset_ = false;
  have_state_ = true;
  last_state_ = go2_nav::BodyState{};
  last_state_.position_m = go2_nav::Vec3{0.0, 0.0, 0.0};
  last_state_.yaw_rad = 0.0;
  last_state_.velocity_mps = go2_nav::Vec2{0.0, 0.0};
  last_state_.yaw_rate_radps = 0.0;
  last_state_.stamp_s = now_s;
  last_state_.valid = true;
  reason_ = go2_nav::VinsReason::Ok;
  if (reason != nullptr) {
    *reason = go2_nav::VinsReason::Ok;
  }
  return true;
}

go2_nav::VinsWireState BridgeCore::makeState(double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  go2_nav::VinsWireState out;
  out.received = true;
  out.seq = ++seq_;
  out.stamp_s = now_s;
  out.measured = config_.extrinsic.measured;

  auto invalid = [&](go2_nav::VinsReason why) {
    out.valid = false;
    out.reason = why;
    return out;
  };

  if (!std::isfinite(now_s)) {
    return invalid(go2_nav::VinsReason::NonFinite);
  }
  if (header_reset_) {
    return invalid(go2_nav::VinsReason::Reset);
  }
  if (!has_sample_) {
    return invalid(go2_nav::VinsReason::NoVins);
  }
  if (!nav_established_ || !estimator_.has_value()) {
    return invalid(go2_nav::VinsReason::NoNav);
  }
  const double age_s = now_s - latest_.receive_stamp_s;
  if (age_s > config_.vins_timeout_s || age_s < -config_.future_skew_s) {
    return invalid(go2_nav::VinsReason::Stale);
  }
  if (!have_state_ || reason_ != go2_nav::VinsReason::Ok) {
    return invalid(reason_ == go2_nav::VinsReason::Ok
                       ? go2_nav::VinsReason::NoVins
                       : reason_);
  }

  out.valid = true;
  out.reason = go2_nav::VinsReason::Ok;
  out.x = last_state_.position_m.x;
  out.y = last_state_.position_m.y;
  out.z = last_state_.position_m.z;
  out.yaw = last_state_.yaw_rad;
  out.vx = last_state_.velocity_mps.x;
  out.vy = last_state_.velocity_mps.y;
  out.wz = last_state_.yaw_rate_radps;
  return out;
}

bool BridgeCore::hasNav() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return nav_established_;
}

Diagnostics BridgeCore::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  Diagnostics diagnostics;
  diagnostics.has_sample = has_sample_;
  diagnostics.has_nav = nav_established_;
  diagnostics.header_stamp_s = latest_.header_stamp_s;
  diagnostics.receive_stamp_s = latest_.receive_stamp_s;
  diagnostics.reason = reason_;
  return diagnostics;
}

}  // namespace go2_vins_bridge
