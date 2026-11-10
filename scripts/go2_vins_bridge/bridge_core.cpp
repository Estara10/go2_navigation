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

const char* healthStateName(HealthState state) {
  switch (state) {
    case HealthState::Uninitialized: return "UNINITIALIZED";
    case HealthState::Healthy: return "HEALTHY";
    case HealthState::Failed: return "FAILED";
    case HealthState::Recovering: return "RECOVERING";
    case HealthState::RecoveryConfirmed: return "RECOVERY_CONFIRMED";
  }
  return "UNKNOWN";
}

BridgeCore::BridgeCore(BridgeConfig config) : config_(config) {}

void BridgeCore::onVinsSample(const VinsSample& sample, double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(now_s) || !finiteSample(sample)) {
    reason_ = go2_nav::VinsReason::NonFinite;
    health_ = HealthState::Failed;
    return;
  }

  if (has_sample_ &&
      sample.header_stamp_s < last_header_stamp_s_ - 1e-9) {
    header_reset_ = true;
    nav_established_ = false;
    estimator_.reset();
    have_state_ = false;
    reason_ = go2_nav::VinsReason::Reset;
    health_ = HealthState::Failed;
  } else if (header_reset_ &&
             sample.header_stamp_s > last_header_stamp_s_ + 1e-9) {
    // A new forward header sequence arrived. The old nav frame is still
    // invalid; the native side must issue SET_ORIGIN again.
    header_reset_ = false;
  }

  if (!has_sample_ || sample.header_stamp_s > last_header_stamp_s_ + 1e-9) {
    last_header_progress_receive_s_ = sample.receive_stamp_s;
  }

  latest_ = sample;
  latest_.camera.stamp_s = sample.receive_stamp_s;
  has_sample_ = true;
  last_header_stamp_s_ = sample.header_stamp_s;

  if (header_reset_) {
    reason_ = go2_nav::VinsReason::Reset;
    health_ = HealthState::Failed;
    return;
  }
  if (!nav_established_ || !estimator_.has_value()) {
    reason_ = go2_nav::VinsReason::NoNav;
    health_ = HealthState::Uninitialized;
    return;
  }

  const double age_s = now_s - sample.receive_stamp_s;
  if (age_s > config_.vins_timeout_s || age_s < -config_.future_skew_s) {
    reason_ = go2_nav::VinsReason::Stale;
    health_ = HealthState::Failed;
    return;
  }
  if (have_feature_sample_ &&
      (sample.receive_stamp_s - last_feature_progress_receive_s_ >
           config_.feature_timeout_s ||
       feature_count_ < config_.minimum_feature_count)) {
    reason_ = go2_nav::VinsReason::Jump;
    health_ = HealthState::Failed;
    return;
  }

  const go2_nav::BodyState state =
      estimator_->update(latest_.camera, now_s);
  if (!state.valid) {
    reason_ = go2_nav::VinsReason::Jump;
    health_ = HealthState::Failed;
    return;
  }

  // Detect pose drift that is not represented by the VINS body twist. This
  // is deliberately a loose 3 s accumulated check; it catches slow pose-only
  // creep without treating single-frame velocity noise as a fault.
  if (have_state_) {
    const double dt = sample.receive_stamp_s - last_state_.stamp_s;
    if (dt > 0.0 && dt <= config_.slow_drift_window_s) {
      const double yaw = 0.5 * (last_state_.yaw_rad + state.yaw_rad);
      const double vx = 0.5 * (last_state_.velocity_mps.x +
                               state.velocity_mps.x);
      const double vy = 0.5 * (last_state_.velocity_mps.y +
                               state.velocity_mps.y);
      const go2_nav::Vec2 predicted{(std::cos(yaw) * vx - std::sin(yaw) * vy) * dt,
                                    (std::sin(yaw) * vx + std::cos(yaw) * vy) * dt};
      const go2_nav::Vec2 observed{state.position_m.x - last_state_.position_m.x,
                                   state.position_m.y - last_state_.position_m.y};
      if (!have_drift_window_) {
        have_drift_window_ = true;
        drift_window_start_s_ = last_state_.stamp_s;
        drift_residual_m_ = go2_nav::Vec2{};
      }
      drift_residual_m_.x += observed.x - predicted.x;
      drift_residual_m_.y += observed.y - predicted.y;
      if (sample.receive_stamp_s - drift_window_start_s_ >=
              config_.slow_drift_window_s &&
          std::hypot(drift_residual_m_.x, drift_residual_m_.y) >
              config_.slow_drift_error_m) {
        have_drift_window_ = false;
        drift_residual_m_ = go2_nav::Vec2{};
        reason_ = go2_nav::VinsReason::Jump;
        health_ = HealthState::Failed;
        return;
      }
      if (sample.receive_stamp_s - drift_window_start_s_ >=
          config_.slow_drift_window_s) {
        drift_window_start_s_ = sample.receive_stamp_s;
        drift_residual_m_ = go2_nav::Vec2{};
      }
    } else {
      have_drift_window_ = false;
      drift_residual_m_ = go2_nav::Vec2{};
    }
  }

  last_state_ = state;
  have_state_ = true;
  if (health_ == HealthState::Failed || health_ == HealthState::Recovering) {
    if (health_ != HealthState::Recovering) {
      health_ = HealthState::Recovering;
      recovery_start_s_ = sample.receive_stamp_s;
    }
    if (sample.receive_stamp_s - recovery_start_s_ >=
        config_.recovery_confirm_s) {
      health_ = HealthState::RecoveryConfirmed;
      reason_ = go2_nav::VinsReason::Ok;
    } else {
      reason_ = go2_nav::VinsReason::Stale;
    }
  } else if (health_ != HealthState::RecoveryConfirmed) {
    health_ = HealthState::Healthy;
    reason_ = go2_nav::VinsReason::Ok;
  } else {
    reason_ = go2_nav::VinsReason::Ok;
  }
}

void BridgeCore::onFeatureCount(int count, double header_stamp_s,
                                double receive_stamp_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count < 0 || !std::isfinite(header_stamp_s) ||
      !std::isfinite(receive_stamp_s)) {
    return;
  }
  if (!have_feature_sample_ ||
      header_stamp_s > feature_header_stamp_s_ + 1e-9) {
    last_feature_progress_receive_s_ = receive_stamp_s;
    feature_header_stamp_s_ = header_stamp_s;
  }
  have_feature_sample_ = true;
  feature_count_ = count;
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
  if (have_feature_sample_ &&
      (now_s - last_feature_progress_receive_s_ > config_.feature_timeout_s ||
       feature_count_ < config_.minimum_feature_count)) {
      return fail(feature_count_ < config_.minimum_feature_count
                    ? go2_nav::VinsReason::Jump
                    : go2_nav::VinsReason::Stale);
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
  health_ = HealthState::Healthy;
  have_drift_window_ = false;
  drift_residual_m_ = go2_nav::Vec2{};
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
  if (has_sample_ && nav_established_ &&
      now_s - last_header_progress_receive_s_ > config_.header_stall_s) {
    health_ = HealthState::Failed;
    reason_ = go2_nav::VinsReason::Stale;
    return invalid(go2_nav::VinsReason::Stale);
  }
  if (have_feature_sample_ &&
      (now_s - last_feature_progress_receive_s_ > config_.feature_timeout_s ||
       feature_count_ < config_.minimum_feature_count)) {
    health_ = HealthState::Failed;
    reason_ = go2_nav::VinsReason::Jump;
    return invalid(go2_nav::VinsReason::Jump);
  }
  if (!has_sample_) {
    return invalid(go2_nav::VinsReason::NoVins);
  }
  if (!nav_established_ || !estimator_.has_value()) {
    return invalid(go2_nav::VinsReason::NoNav);
  }
  const double age_s = now_s - latest_.receive_stamp_s;
  if (age_s > config_.vins_timeout_s || age_s < -config_.future_skew_s) {
    health_ = HealthState::Failed;
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
  diagnostics.health = health_;
  diagnostics.feature_count = feature_count_;
  return diagnostics;
}

}  // namespace go2_vins_bridge
