#include "body_state.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace g = go2_nav;

namespace {

int failures = 0;

void check(bool condition, const std::string& name) {
  if (!condition) {
    std::cerr << "FAIL " << name << "\n";
    ++failures;
  }
}

void checkNear(double actual, double expected, const std::string& name,
               double tolerance = 1e-9) {
  check(std::abs(actual - expected) <= tolerance, name);
}

void checkVec2(const g::Vec2& actual, double x, double y,
               const std::string& name, double tolerance = 1e-9) {
  checkNear(actual.x, x, name + " x", tolerance);
  checkNear(actual.y, y, name + " y", tolerance);
}

void checkVec3(const g::Vec3& actual, double x, double y, double z,
               const std::string& name, double tolerance = 1e-9) {
  checkNear(actual.x, x, name + " x", tolerance);
  checkNear(actual.y, y, name + " y", tolerance);
  checkNear(actual.z, z, name + " z", tolerance);
}

void checkMat3Equal(const g::Mat3& actual, const g::Mat3& expected,
                    const std::string& name, double tolerance = 1e-9) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      checkNear(actual(row, col), expected(row, col),
                name + " element " + std::to_string(row) + "," +
                    std::to_string(col),
                tolerance);
    }
  }
}

void checkMat3Identity(const g::Mat3& actual, const std::string& name,
                       double tolerance = 1e-9) {
  checkMat3Equal(actual, g::Mat3::identity(), name, tolerance);
}

g::Pose makePose(double x, double y, double z, double yaw,
                 double pitch = 0.0, double roll = 0.0) {
  g::Pose pose;
  pose.position = g::Vec3{x, y, z};
  pose.rotation = g::Mat3::fromYawPitchRoll(yaw, pitch, roll);
  return pose;
}

g::CameraState makeCamera(double stamp_s, const g::Pose& world_camera,
                          const g::Twist& twist = g::Twist{}) {
  g::CameraState camera;
  camera.stamp_s = stamp_s;
  camera.world_camera = world_camera;
  camera.body_twist_camera = twist;
  return camera;
}

void testMathBasics() {
  const int before = failures;
  checkVec3(g::Mat3::identity().apply(g::Vec3{1.0, 2.0, 3.0}), 1.0, 2.0, 3.0,
            "identity apply");
  checkVec3(g::Mat3::rotationZ(g::kPi / 2.0).apply(g::Vec3{1.0, 0.0, 0.0}),
            0.0, 1.0, 0.0, "rotationZ 90 applies to +X");
  checkMat3Identity(g::Mat3::fromYawPitchRoll(0.0, 0.0, 0.0),
                    "fromYawPitchRoll zero");
  checkNear(g::wrapPi(g::kPi), g::kPi, "wrap +pi");
  checkNear(g::wrapPi(-g::kPi), g::kPi, "wrap -pi");
  checkNear(g::wrapPi(3.0 * g::kPi / 2.0), -g::kPi / 2.0, "wrap 3pi/2");
  std::cout << (failures == before ? "PASS" : "FAIL") << " math_basics\n";
}

void testIdentityExtrinsic() {
  const int before = failures;
  g::Extrinsic extrinsic;
  check(!extrinsic.measured, "default extrinsic is unmeasured");

  const g::Pose camera_pose = makePose(1.0, 2.0, 3.0, 0.7, 0.1, -0.05);
  const g::Twist camera_twist{g::Vec3{0.1, 0.2, 0.0},
                              g::Vec3{0.0, 0.0, 0.3}};

  const g::Pose base_pose = g::basePoseFromCamera(camera_pose, extrinsic);
  checkVec3(base_pose.position, 1.0, 2.0, 3.0, "identity base position");
  checkMat3Equal(base_pose.rotation, camera_pose.rotation,
                 "identity base rotation matrix");

  const g::Twist base_twist =
      g::baseTwistFromCamera(camera_twist, extrinsic);
  checkVec3(base_twist.linear, 0.1, 0.2, 0.0, "identity base linear velocity");
  checkVec3(base_twist.angular, 0.0, 0.0, 0.3,
            "identity base angular velocity");

  const g::NavFrame nav = g::NavFrame::fromInitialBasePose(base_pose);
  g::BodyStateEstimator estimator(nav, extrinsic);
  const g::BodyState state =
      estimator.update(makeCamera(10.0, camera_pose, camera_twist), 10.0);
  check(state.valid, "identity estimator valid");
  checkVec3(state.position_m, 0.0, 0.0, 0.0, "identity nav position");
  checkNear(state.yaw_rad, 0.0, "identity nav yaw");
  checkVec2(state.velocity_mps, 0.1, 0.2, "identity body velocity");
  checkNear(state.yaw_rate_radps, 0.3, "identity yaw rate");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " identity_extrinsic\n";
}

void testPureTranslation() {
  const int before = failures;
  g::Extrinsic extrinsic;
  extrinsic.t_base_cam = g::Vec3{0.1, 0.0, 0.0};

  const g::Pose camera_at_origin = makePose(0.0, 0.0, 0.0, 0.0);
  const g::Pose base_at_origin =
      g::basePoseFromCamera(camera_at_origin, extrinsic);
  checkVec3(base_at_origin.position, -0.1, 0.0, 0.0,
            "translation base initial position");
  checkMat3Identity(base_at_origin.rotation, "translation base rotation");

  const g::Pose camera_moved = makePose(0.1, 0.0, 0.0, 0.0);
  const g::Pose base_moved = g::basePoseFromCamera(camera_moved, extrinsic);
  checkVec3(base_moved.position, 0.0, 0.0, 0.0,
            "translation base moved position");

  const g::NavFrame nav = g::NavFrame::fromInitialBasePose(base_at_origin);
  g::BodyStateEstimator estimator(nav, extrinsic);
  const g::Twist camera_twist{g::Vec3{0.5, 0.0, 0.0}, g::Vec3{}};
  const g::BodyState first =
      estimator.update(makeCamera(1.0, camera_at_origin, camera_twist), 1.0);
  check(first.valid, "translation first sample valid");
  checkVec3(first.position_m, 0.0, 0.0, 0.0,
            "translation first nav position");

  const g::BodyState second =
      estimator.update(makeCamera(1.1, camera_moved, camera_twist), 1.1);
  check(second.valid, "translation second sample valid");
  checkVec3(second.position_m, 0.1, 0.0, 0.0,
            "translation second nav position");
  checkVec2(second.velocity_mps, 0.5, 0.0,
            "translation body velocity");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " pure_translation\n";
}

void testPureRotation() {
  const int before = failures;
  g::Extrinsic extrinsic;
  extrinsic.R_base_cam = g::Mat3::rotationZ(g::kPi / 2.0);

  const g::Pose camera_identity = makePose(0.0, 0.0, 0.0, 0.0);
  const g::Pose base_from_identity =
      g::basePoseFromCamera(camera_identity, extrinsic);
  checkNear(std::atan2(base_from_identity.rotation(1, 0),
                       base_from_identity.rotation(0, 0)),
            -g::kPi / 2.0, "rotation base yaw is -90 deg");

  const g::Pose camera_rotated = makePose(0.0, 0.0, 0.0, g::kPi / 2.0);
  const g::Pose base_from_rotated =
      g::basePoseFromCamera(camera_rotated, extrinsic);
  checkNear(std::atan2(base_from_rotated.rotation(1, 0),
                       base_from_rotated.rotation(0, 0)),
            0.0, "rotation base yaw is 0 when camera is rotated");
  checkMat3Identity(base_from_rotated.rotation,
                    "rotation base matrix is identity", 1e-12);

  const g::Mat3 arbitrary =
      g::Mat3::fromYawPitchRoll(0.3, 0.1, -0.05);
  g::Extrinsic arbitrary_extrinsic;
  arbitrary_extrinsic.R_base_cam = arbitrary;
  g::Pose camera_arbitrary = makePose(0.0, 0.0, 0.0, 0.0);
  camera_arbitrary.rotation = arbitrary;
  const g::Pose base_arbitrary =
      g::basePoseFromCamera(camera_arbitrary, arbitrary_extrinsic);
  checkMat3Identity(base_arbitrary.rotation,
                    "arbitrary rotation base matrix is identity", 1e-12);

  const g::Twist camera_twist{g::Vec3{1.0, 0.0, 0.0},
                              g::Vec3{0.0, 0.0, 1.0}};
  const g::Twist base_twist =
      g::baseTwistFromCamera(camera_twist, extrinsic);
  checkVec3(base_twist.linear, 0.0, 1.0, 0.0,
            "rotation base linear velocity");
  checkVec3(base_twist.angular, 0.0, 0.0, 1.0,
            "rotation base angular velocity");
  std::cout << (failures == before ? "PASS" : "FAIL") << " pure_rotation\n";
}

void testLeverArm() {
  const int before = failures;
  g::Extrinsic extrinsic;
  extrinsic.t_base_cam = g::Vec3{1.0, 0.0, 0.0};

  const g::Twist camera_twist{g::Vec3{0.0, 2.0, 0.0},
                              g::Vec3{0.0, 0.0, 2.0}};
  const g::Twist base_twist =
      g::baseTwistFromCamera(camera_twist, extrinsic);
  checkVec3(base_twist.linear, 0.0, 0.0, 0.0,
            "lever arm cancels camera linear velocity", 1e-12);
  checkNear(base_twist.angular.z, 2.0, "lever arm keeps angular velocity");

  g::Extrinsic behind;
  behind.t_base_cam = g::Vec3{-1.0, 0.0, 0.0};
  const g::Twist behind_twist =
      g::baseTwistFromCamera(g::Twist{g::Vec3{0.0, -2.0, 0.0},
                                      g::Vec3{0.0, 0.0, 2.0}},
                             behind);
  checkVec3(behind_twist.linear, 0.0, 0.0, 0.0,
            "lever arm cancels camera behind base", 1e-12);
  std::cout << (failures == before ? "PASS" : "FAIL") << " lever_arm\n";
}

void testTranslationRotationCombined() {
  const int before = failures;
  g::Extrinsic extrinsic;
  extrinsic.R_base_cam = g::Mat3::rotationZ(g::kPi / 2.0);
  extrinsic.t_base_cam = g::Vec3{1.0, 2.0, 0.0};

  const g::Pose camera = makePose(10.0, 20.0, 0.0, 30.0 * g::kPi / 180.0);
  const g::Pose base = g::basePoseFromCamera(camera, extrinsic);
  const double sqrt3_over_2 = std::sqrt(3.0) / 2.0;
  const double expected_x = 10.0 - (0.5 * 1.0 + sqrt3_over_2 * 2.0);
  const double expected_y = 20.0 - (-sqrt3_over_2 * 1.0 + 0.5 * 2.0);
  checkVec3(base.position, expected_x, expected_y, 0.0,
            "combined base position");
  checkNear(std::atan2(base.rotation(1, 0), base.rotation(0, 0)),
            -60.0 * g::kPi / 180.0, "combined base yaw");

  const g::Twist base_twist = g::baseTwistFromCamera(
      g::Twist{g::Vec3{1.0, 0.0, 0.0}, g::Vec3{0.0, 0.0, 1.0}}, extrinsic);
  checkVec3(base_twist.linear, 2.0, 0.0, 0.0,
            "combined base linear velocity");
  checkVec3(base_twist.angular, 0.0, 0.0, 1.0,
            "combined base angular velocity");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " translation_rotation_combined\n";
}

void testNavFrameNonZeroInitialHeading() {
  const int before = failures;
  const g::Pose initial_base =
      makePose(5.0, -2.0, 0.3, g::kPi / 2.0);
  const g::NavFrame nav = g::NavFrame::fromInitialBasePose(initial_base);

  checkVec3(nav.worldToNav(initial_base.position), 0.0, 0.0, 0.0,
            "nav initial position is origin");
  checkVec3(nav.worldToNav(g::Vec3{5.0, -1.0, 0.3}), 1.0, 0.0, 0.0,
            "nav initial heading is +X");
  checkVec3(nav.worldToNav(g::Vec3{4.0, -2.0, 0.3}), 0.0, 1.0, 0.0,
            "nav left is +Y");
  checkVec3(nav.navToWorld(g::Vec3{1.0, 0.0, 0.0}), 5.0, -1.0, 0.3,
            "nav round trip");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " nav_frame_nonzero_heading\n";
}

void testRelativeTargetOnce() {
  const int before = failures;
  g::BodyState command_state;
  command_state.position_m = g::Vec3{2.0, 3.0, 0.0};
  command_state.yaw_rad = g::kPi / 2.0;

  const g::Vec3 forward =
      g::relativeTargetToNav(command_state, 1.0, 0.0);
  const g::Vec3 left = g::relativeTargetToNav(command_state, 0.0, 1.0);
  checkVec3(forward, 2.0, 4.0, 0.0, "forward target at yaw 90");
  checkVec3(left, 1.0, 3.0, 0.0, "left target at yaw 90");

  g::BodyState after_turn = command_state;
  after_turn.yaw_rad = 0.0;
  const g::Vec3 fixed_again =
      g::relativeTargetToNav(command_state, 1.0, 0.0);
  checkVec3(fixed_again, 2.0, 4.0, 0.0,
            "target is fixed when using command state");
  const g::Vec3 if_recomputed =
      g::relativeTargetToNav(after_turn, 1.0, 0.0);
  checkVec3(if_recomputed, 3.0, 3.0, 0.0,
            "recomputing after a turn would move the target");

  g::BodyState diagonal_state;
  diagonal_state.position_m = g::Vec3{1.0, 1.0, 0.0};
  diagonal_state.yaw_rad = 30.0 * g::kPi / 180.0;
  const double c = std::cos(30.0 * g::kPi / 180.0);
  const double s = std::sin(30.0 * g::kPi / 180.0);
  checkVec3(g::relativeTargetToNav(diagonal_state, 2.0, 0.5),
            1.0 + c * 2.0 - s * 0.5, 1.0 + s * 2.0 + c * 0.5, 0.0,
            "diagonal target at yaw 30");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " relative_target_once\n";
}

void testValidity() {
  const int before = failures;
  const g::Extrinsic extrinsic;
  const g::Pose initial_camera = makePose(0.0, 0.0, 0.0, 0.0);
  const g::Pose initial_base =
      g::basePoseFromCamera(initial_camera, extrinsic);
  const g::NavFrame nav = g::NavFrame::fromInitialBasePose(initial_base);
  g::BodyStateEstimator estimator(nav, extrinsic);

  g::CameraState camera = makeCamera(1.0, initial_camera);
  g::BodyState state = estimator.update(camera, 1.0);
  check(state.valid, "validity initial sample valid");
  check(estimator.hasLastState(), "validity last state stored");

  state = estimator.update(camera, 1.6);
  check(!state.valid, "validity timeout invalid");
  checkNear(estimator.lastState().stamp_s, 1.0,
            "validity timeout does not replace last state");

  camera.stamp_s = 1.7;
  state = estimator.update(camera, 1.0);
  check(!state.valid, "validity future timestamp invalid");

  camera.stamp_s = 1.05;
  camera.world_camera.position.x = std::numeric_limits<double>::quiet_NaN();
  state = estimator.update(camera, 1.05);
  check(!state.valid, "validity non-finite position invalid");
  camera.world_camera.position.x = 0.0;

  camera.valid_input = false;
  state = estimator.update(camera, 1.05);
  check(!state.valid, "validity upstream invalid flag");
  camera.valid_input = true;

  camera.stamp_s = 0.9;
  state = estimator.update(camera, 1.0);
  check(!state.valid, "validity non-monotonic timestamp invalid");

  estimator.reset();
  camera = makeCamera(2.0, initial_camera);
  state = estimator.update(camera, 2.0);
  check(state.valid, "validity reset then first sample valid");
  camera.stamp_s = 2.1;
  camera.world_camera.position.x = 1.0;
  state = estimator.update(camera, 2.1);
  check(!state.valid, "validity position jump invalid");
  checkNear(estimator.lastState().stamp_s, 2.0,
            "validity jump does not replace last state");

  estimator.reset();
  camera = makeCamera(2.2, makePose(1.0, 0.0, 0.0, 0.0));
  state = estimator.update(camera, 2.2);
  check(state.valid, "validity explicit reset recovers");

  estimator.reset();
  camera = makeCamera(3.0, initial_camera);
  state = estimator.update(camera, 3.0);
  check(state.valid, "validity yaw jump first sample valid");
  camera.stamp_s = 3.1;
  camera.world_camera = makePose(0.0, 0.0, 0.0, g::kPi / 2.0);
  state = estimator.update(camera, 3.1);
  check(!state.valid, "validity yaw jump invalid");

  g::ValidityConfig tight;
  tight.max_position_jump_m = 0.05;
  g::BodyStateEstimator tight_estimator(nav, extrinsic, tight);
  state = tight_estimator.update(makeCamera(4.0, initial_camera), 4.0);
  check(state.valid, "validity custom config first sample");
  state = tight_estimator.update(
      makeCamera(4.1, makePose(0.1, 0.0, 0.0, 0.0)), 4.1);
  check(!state.valid, "validity custom jump threshold");
  std::cout << (failures == before ? "PASS" : "FAIL") << " validity\n";
}

void testVelocityCommandDefaults() {
  const int before = failures;
  const g::VelocityCommand command;
  checkNear(command.stamp_s, 0.0, "command stamp default");
  checkNear(command.vx_mps, 0.0, "command vx default");
  checkNear(command.vy_mps, 0.0, "command vy default");
  checkNear(command.vyaw_radps, 0.0, "command vyaw default");
  check(!command.stop_request, "command stop default");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " velocity_command_defaults\n";
}

}  // namespace

int main() {
  testMathBasics();
  testIdentityExtrinsic();
  testPureTranslation();
  testPureRotation();
  testLeverArm();
  testTranslationRotationCombined();
  testNavFrameNonZeroInitialHeading();
  testRelativeTargetOnce();
  testValidity();
  testVelocityCommandDefaults();

  if (failures != 0) {
    std::cerr << "go2_nav_core selftest failures: " << failures << "\n";
    return 1;
  }
  std::cout << "all go2_nav_core selftests passed; "
               "no ROS/DDS/SDK initialized\n";
  return 0;
}
