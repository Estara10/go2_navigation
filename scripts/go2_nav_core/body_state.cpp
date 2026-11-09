#include "body_state.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace go2_nav {

Mat3 Mat3::identity() { return Mat3{}; }

Mat3 Mat3::rotationZ(double yaw_rad) {
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  Mat3 result;
  result.m = {{c, -s, 0.0,
               s,  c, 0.0,
               0.0, 0.0, 1.0}};
  return result;
}

Mat3 Mat3::fromYawPitchRoll(double yaw_rad, double pitch_rad,
                            double roll_rad) {
  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);
  const double cr = std::cos(roll_rad);
  const double sr = std::sin(roll_rad);

  Mat3 result;
  result.m = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
               sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
               -sp,     cp * sr,                cp * cr}};
  return result;
}

double Mat3::operator()(int row, int col) const {
  return m[static_cast<std::size_t>(row * 3 + col)];
}

double& Mat3::operator()(int row, int col) {
  return m[static_cast<std::size_t>(row * 3 + col)];
}

Mat3 Mat3::transposed() const {
  Mat3 result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result(row, col) = (*this)(col, row);
    }
  }
  return result;
}

Vec3 Mat3::apply(const Vec3& vector) const {
  return Vec3{
      (*this)(0, 0) * vector.x + (*this)(0, 1) * vector.y +
          (*this)(0, 2) * vector.z,
      (*this)(1, 0) * vector.x + (*this)(1, 1) * vector.y +
          (*this)(1, 2) * vector.z,
      (*this)(2, 0) * vector.x + (*this)(2, 1) * vector.y +
          (*this)(2, 2) * vector.z};
}

Mat3 Mat3::operator*(const Mat3& rhs) const {
  Mat3 result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int k = 0; k < 3; ++k) {
        value += (*this)(row, k) * rhs(k, col);
      }
      result(row, col) = value;
    }
  }
  return result;
}

Vec3 operator+(const Vec3& lhs, const Vec3& rhs) {
  return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
  return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(double scalar, const Vec3& vector) {
  return Vec3{scalar * vector.x, scalar * vector.y, scalar * vector.z};
}

double dot(const Vec3& lhs, const Vec3& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
  return Vec3{lhs.y * rhs.z - lhs.z * rhs.y,
              lhs.z * rhs.x - lhs.x * rhs.z,
              lhs.x * rhs.y - lhs.y * rhs.x};
}

double norm(const Vec3& vector) { return std::sqrt(dot(vector, vector)); }

bool isFinite(double value) { return std::isfinite(value); }

bool isFinite(const Vec3& vector) {
  return isFinite(vector.x) && isFinite(vector.y) && isFinite(vector.z);
}

bool isFinite(const Mat3& matrix) {
  for (const double value : matrix.m) {
    if (!isFinite(value)) {
      return false;
    }
  }
  return true;
}

double wrapPi(double angle) {
  angle = std::fmod(angle, 2.0 * kPi);
  if (angle <= -kPi) {
    angle += 2.0 * kPi;
  } else if (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  return angle;
}

Pose basePoseFromCamera(const Pose& world_camera, const Extrinsic& extrinsic) {
  Pose base;
  base.rotation = world_camera.rotation * extrinsic.R_base_cam.transposed();
  base.position =
      world_camera.position - base.rotation.apply(extrinsic.t_base_cam);
  return base;
}

Twist baseTwistFromCamera(const Twist& camera_twist,
                          const Extrinsic& extrinsic) {
  Twist base;
  base.angular = extrinsic.R_base_cam.apply(camera_twist.angular);
  base.linear = extrinsic.R_base_cam.apply(camera_twist.linear) -
                cross(base.angular, extrinsic.t_base_cam);
  return base;
}

NavFrame NavFrame::fromInitialBasePose(const Pose& world_base) {
  NavFrame frame;
  const double yaw = std::atan2(world_base.rotation(1, 0),
                                world_base.rotation(0, 0));
  frame.origin_world_ = world_base.position;
  frame.R_world_nav_ = Mat3::rotationZ(yaw);
  return frame;
}

Vec3 NavFrame::worldToNav(const Vec3& world_position) const {
  return R_world_nav_.transposed().apply(world_position - origin_world_);
}

Vec3 NavFrame::navToWorld(const Vec3& nav_position) const {
  return origin_world_ + R_world_nav_.apply(nav_position);
}

BodyStateEstimator::BodyStateEstimator(NavFrame nav_frame, Extrinsic extrinsic,
                                       ValidityConfig config)
    : nav_frame_(std::move(nav_frame)),
      extrinsic_(extrinsic),
      config_(config) {}

BodyState BodyStateEstimator::update(const CameraState& camera, double now_s) {
  BodyState state;
  state.stamp_s = camera.stamp_s;

  const bool input_finite =
      isFinite(now_s) && isFinite(camera.stamp_s) &&
      isFinite(camera.world_camera.position) &&
      isFinite(camera.world_camera.rotation) &&
      isFinite(camera.body_twist_camera.linear) &&
      isFinite(camera.body_twist_camera.angular);
  if (!camera.valid_input || !input_finite) {
    return state;
  }

  const double age_s = now_s - camera.stamp_s;
  if (age_s > config_.max_age_s || age_s < -config_.max_future_skew_s) {
    return state;
  }

  const Pose world_base =
      basePoseFromCamera(camera.world_camera, extrinsic_);
  const Twist base_twist =
      baseTwistFromCamera(camera.body_twist_camera, extrinsic_);
  const Mat3 R_nav_base =
      nav_frame_.worldFromNav().transposed() * world_base.rotation;

  state.position_m = nav_frame_.worldToNav(world_base.position);
  state.yaw_rad = std::atan2(R_nav_base(1, 0), R_nav_base(0, 0));
  state.velocity_mps = Vec2{base_twist.linear.x, base_twist.linear.y};
  state.yaw_rate_radps = base_twist.angular.z;

  if (!isFinite(state.position_m) || !isFinite(state.yaw_rad) ||
      !isFinite(state.velocity_mps.x) || !isFinite(state.velocity_mps.y) ||
      !isFinite(state.yaw_rate_radps)) {
    return state;
  }

  if (has_last_) {
    if (camera.stamp_s < last_.stamp_s) {
      return state;
    }
    const double position_jump = norm(state.position_m - last_.position_m);
    const double yaw_jump = std::abs(wrapPi(state.yaw_rad - last_.yaw_rad));
    if (position_jump > config_.max_position_jump_m ||
        yaw_jump > config_.max_yaw_jump_rad) {
      return state;
    }
  }

  state.valid = true;
  last_ = state;
  has_last_ = true;
  return state;
}

void BodyStateEstimator::reset() {
  has_last_ = false;
  last_ = BodyState{};
}

Vec3 relativeTargetToNav(const BodyState& state_at_command, double forward_m,
                         double left_m) {
  const double c = std::cos(state_at_command.yaw_rad);
  const double s = std::sin(state_at_command.yaw_rad);
  return Vec3{state_at_command.position_m.x + c * forward_m - s * left_m,
              state_at_command.position_m.y + s * forward_m + c * left_m,
              state_at_command.position_m.z};
}

}  // namespace go2_nav
