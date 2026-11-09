#ifndef GO2_NAV_CORE_BODY_STATE_HPP
#define GO2_NAV_CORE_BODY_STATE_HPP

#include <array>

namespace go2_nav {

inline constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// Row-major 3x3 matrix. Default construction is the identity matrix.
struct Mat3 {
  std::array<double, 9> m{{1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0}};

  static Mat3 identity();
  static Mat3 rotationZ(double yaw_rad);
  // R = Rz(yaw) * Ry(pitch) * Rx(roll); used by tests and configuration.
  static Mat3 fromYawPitchRoll(double yaw_rad, double pitch_rad,
                               double roll_rad);

  double operator()(int row, int col) const;
  double& operator()(int row, int col);
  Mat3 transposed() const;
  Vec3 apply(const Vec3& vector) const;
  Mat3 operator*(const Mat3& rhs) const;
};

Vec3 operator+(const Vec3& lhs, const Vec3& rhs);
Vec3 operator-(const Vec3& lhs, const Vec3& rhs);
Vec3 operator*(double scalar, const Vec3& vector);
double dot(const Vec3& lhs, const Vec3& rhs);
Vec3 cross(const Vec3& lhs, const Vec3& rhs);
double norm(const Vec3& vector);

bool isFinite(double value);
bool isFinite(const Vec3& vector);
bool isFinite(const Mat3& matrix);
double wrapPi(double angle);

// Pose of a frame expressed in its parent frame:
//   p_parent = R_parent_frame * p_frame + p_parent_frame
struct Pose {
  Vec3 position;
  Mat3 rotation = Mat3::identity();
};

// Linear and angular velocity of a rigid body, both expressed in the same
// body frame.
struct Twist {
  Vec3 linear;
  Vec3 angular;
};

// Rigid transform from camera IMU coordinates to base_link coordinates:
//   p_base = R_base_cam * p_cam + t_base_cam
// t_base_cam is the camera IMU frame origin expressed in base_link.
// R_base_cam maps camera-frame vectors into base_link coordinates; its columns
// are the camera axes expressed in base_link.
//
// measured=false is the default. Do not use an unmeasured/synthetic transform
// for robot control; see docs/INTERFACE.md and docs/MEASUREMENT_CHECKLIST.md.
struct Extrinsic {
  Mat3 R_base_cam = Mat3::identity();
  Vec3 t_base_cam{};
  bool measured = false;
};

struct CameraState {
  double stamp_s = 0.0;
  Pose world_camera;              // camera IMU pose in the source/VINS world
  Twist body_twist_camera;        // linear/angular in the camera IMU frame
  bool valid_input = true;        // upstream tracking/reset validity flag
};

struct BodyState {
  double stamp_s = 0.0;
  Vec3 position_m;                // base_link origin in nav
  double yaw_rad = 0.0;           // base_link +X relative to nav +X, left positive
  Vec2 velocity_mps;              // base_link origin velocity in base_link (vx, vy)
  double yaw_rate_radps = 0.0;    // angular velocity Z in base_link, left positive
  bool valid = false;
};

struct VelocityCommand {
  double stamp_s = 0.0;
  double vx_mps = 0.0;            // forward in base_link
  double vy_mps = 0.0;            // left in base_link
  double vyaw_radps = 0.0;        // left positive
  bool stop_request = false;
};

// Rigid-transform helpers. Both are pure functions.
Pose basePoseFromCamera(const Pose& world_camera, const Extrinsic& extrinsic);
Twist baseTwistFromCamera(const Twist& camera_twist, const Extrinsic& extrinsic);

class NavFrame {
 public:
  // The nav origin is the horizontal projection of the initial base_link
  // origin. The vertical origin is the initial base_link height because this
  // module does not estimate ground height; horizontal x/y are unaffected.
  static NavFrame fromInitialBasePose(const Pose& world_base);

  Vec3 worldToNav(const Vec3& world_position) const;
  Vec3 navToWorld(const Vec3& nav_position) const;

  const Vec3& originWorld() const { return origin_world_; }
  const Mat3& worldFromNav() const { return R_world_nav_; }

 private:
  Vec3 origin_world_;
  Mat3 R_world_nav_ = Mat3::identity();
};

// Initial offline defaults. They are configurable and are not field-validated
// safety limits.
struct ValidityConfig {
  double max_age_s = 0.5;
  double max_position_jump_m = 0.25;
  double max_yaw_jump_rad = 30.0 * kPi / 180.0;
  double max_future_skew_s = 0.05;
};

class BodyStateEstimator {
 public:
  BodyStateEstimator(NavFrame nav_frame, Extrinsic extrinsic,
                     ValidityConfig config = {});

  // Returns a BodyState with valid=false on any invalid input, timeout,
  // non-monotonic timestamp, or jump. The last accepted state is not replaced
  // by an invalid sample; call reset() after an acknowledged source reset.
  BodyState update(const CameraState& camera, double now_s);

  void reset();
  bool hasLastState() const { return has_last_; }
  const BodyState& lastState() const { return last_; }
  const Extrinsic& extrinsic() const { return extrinsic_; }
  const NavFrame& navFrame() const { return nav_frame_; }

 private:
  NavFrame nav_frame_;
  Extrinsic extrinsic_;
  ValidityConfig config_;
  bool has_last_ = false;
  BodyState last_;
};

// Pure function: converts a target given relative to the base_link pose at
// command time into a fixed nav point. It must be called once at command time;
// later base_link motion must not change the returned point.
Vec3 relativeTargetToNav(const BodyState& state_at_command, double forward_m,
                         double left_m);

}  // namespace go2_nav

#endif
