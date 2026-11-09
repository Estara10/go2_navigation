#include "path.hpp"
#include "pure_pursuit.hpp"
#include "vins_replay.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

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

void checkVec3(const g::Vec3& actual, double x, double y, double z,
               const std::string& name, double tolerance = 1e-9) {
  checkNear(actual.x, x, name + " x", tolerance);
  checkNear(actual.y, y, name + " y", tolerance);
  checkNear(actual.z, z, name + " z", tolerance);
}

struct SimulationResult {
  bool stopped = false;
  bool all_vy_zero = true;
  double final_error_m = 0.0;
  double final_yaw_error_rad = 0.0;
  double max_yaw_rate_radps = 0.0;
  double time_s = 0.0;
};

SimulationResult simulate(const g::Path& path, const g::Pose2D& start,
                          const g::PurePursuitConfig& config,
                          double dt = 0.005, double max_time_s = 60.0) {
  SimulationResult result;
  g::PurePursuit tracker(config);
  g::PurePursuitState tracker_state;
  g::BodyState state;
  state.position_m = g::Vec3{start.x, start.y, 0.0};
  state.yaw_rad = start.yaw;
  state.valid = true;
  state.stamp_s = 0.0;

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
    state.yaw_rad =
        g::wrapPi(state.yaw_rad + command.vyaw_radps * dt);
    state.velocity_mps = g::Vec2{command.vx_mps, 0.0};
    state.yaw_rate_radps = command.vyaw_radps;
    state.stamp_s += dt;
  }

  const g::PathPoint goal = path.points().back();
  result.final_error_m =
      std::hypot(state.position_m.x - goal.pose.x,
                 state.position_m.y - goal.pose.y);
  result.final_yaw_error_rad =
      std::abs(g::wrapPi(state.yaw_rad - goal.pose.yaw));
  return result;
}

void testStraightPath() {
  const g::Pose2D start{0.0, 0.0, 0.0};
  const g::Pose2D goal{1.0, 0.0, 0.0};
  const g::Path path = g::makeStraightPath(start, goal);
  check(!path.empty(), "straight path not empty");
  checkNear(path.length(), 1.0, "straight path length", 1e-9);
  checkNear(path.points().front().pose.x, 0.0, "straight start x");
  checkNear(path.points().back().pose.x, 1.0, "straight end x");
  const g::PathPoint middle = path.sample(0.5);
  checkNear(middle.pose.x, 0.5, "straight sample x", 1e-9);
  checkNear(middle.pose.y, 0.0, "straight sample y", 1e-9);
  double distance = 0.0;
  const double projected =
      path.project(g::Vec2{0.5, 0.1}, &distance);
  checkNear(projected, 0.5, "straight projection s", 1e-9);
  checkNear(distance, 0.1, "straight projection distance", 1e-9);
}

void testDubinsPath() {
  const g::Pose2D start{0.0, 0.0, 0.0};
  const g::Pose2D goal{1.0, 1.0, g::kPi / 2.0};
  const g::Path path = g::makeDubinsPath(start, goal, 0.5);
  check(!path.empty(), "dubins path not empty");
  check(path.length() > 0.0, "dubins path length positive");
  const g::PathPoint& last = path.points().back();
  checkNear(last.pose.x, goal.x, "dubins end x", 1e-6);
  checkNear(last.pose.y, goal.y, "dubins end y", 1e-6);
  checkNear(g::wrapPi(last.pose.yaw - goal.yaw), 0.0, "dubins end yaw", 1e-6);

  const g::Pose2D behind_start{0.0, 0.0, 0.0};
  const g::Pose2D behind_goal{-0.5, 0.0, 0.0};
  const g::Path behind = g::makeDubinsPath(behind_start, behind_goal, 0.5);
  check(!behind.empty(), "dubins behind path not empty");
  checkNear(behind.points().back().pose.x, behind_goal.x,
            "dubins behind end x", 1e-6);
  checkNear(behind.points().back().pose.y, behind_goal.y,
            "dubins behind end y", 1e-6);
}

void testMakePathStraight() {
  const g::Pose2D start{0.0, 0.0, 0.0};
  const g::Pose2D goal{1.0, 0.0, 0.0};
  const g::Path path = g::makePath(start, goal, 0.5);
  check(!path.empty(), "makePath straight not empty");
  checkNear(path.length(), 1.0, "makePath straight length", 1e-9);
}

void testPurePursuitStraight() {
  const g::Pose2D start{0.0, 0.0, 0.0};
  const g::Pose2D goal{1.0, 0.0, 0.0};
  const g::Path path = g::makeStraightPath(start, goal);

  g::PurePursuitConfig config;
  config.lookahead_min_m = 0.20;
  config.lookahead_gain_s = 1.0;
  config.lookahead_max_m = 0.60;
  config.max_speed_mps = 0.30;
  config.min_speed_mps = 0.03;
  config.max_yaw_rate_radps = 0.50;
  config.max_lateral_accel_mps2 = 0.50;
  config.decel_mps2 = 0.30;
  config.stop_distance_m = 0.02;
  config.goal_tolerance_m = 0.02;

  const SimulationResult result = simulate(path, start, config);
  check(result.stopped, "straight simulation stopped");
  check(result.final_error_m < 0.05, "straight simulation final error < 5 cm");
  check(result.all_vy_zero, "straight simulation vy is zero");
  check(result.max_yaw_rate_radps <= config.max_yaw_rate_radps + 1e-9,
        "straight simulation yaw rate limited");
  check(result.time_s < 15.0, "straight simulation finishes in time");
}

void testPurePursuitCurve() {
  const g::Pose2D start{0.0, 0.0, 0.0};
  const g::Pose2D goal{1.0, 1.0, g::kPi / 2.0};
  const g::Path path = g::makeDubinsPath(start, goal, 0.5);

  g::PurePursuitConfig config;
  config.lookahead_min_m = 0.20;
  config.lookahead_gain_s = 1.0;
  config.lookahead_max_m = 0.60;
  config.max_speed_mps = 0.20;
  config.min_speed_mps = 0.03;
  config.max_yaw_rate_radps = 0.50;
  config.max_lateral_accel_mps2 = 0.50;
  config.decel_mps2 = 0.30;
  config.stop_distance_m = 0.02;
  config.goal_tolerance_m = 0.02;

  const SimulationResult result = simulate(path, start, config);
  check(result.stopped, "curve simulation stopped");
  check(result.final_error_m < 0.05, "curve simulation final error < 5 cm");
  check(result.all_vy_zero, "curve simulation vy is zero");
  check(result.max_yaw_rate_radps <= config.max_yaw_rate_radps + 1e-9,
        "curve simulation yaw rate limited");
  check(result.time_s < 30.0, "curve simulation finishes in time");
}

void testTargetSemantics() {
  g::BodyState state;
  state.position_m = g::Vec3{0.0, 0.0, 0.0};
  state.yaw_rad = g::kPi / 2.0;
  const g::Vec3 target = g::relativeTargetToNav(state, 1.0, 0.5);
  checkVec3(target, -0.5, 1.0, 0.0, "relative target at 90 deg", 1e-9);

  g::BodyState turned = state;
  turned.yaw_rad = 0.0;
  const g::Vec3 other = g::relativeTargetToNav(turned, 1.0, 0.5);
  checkVec3(other, 1.0, 0.5, 0.0, "relative target at 0 deg", 1e-9);
  check(std::abs(target.x - other.x) > 1e-6,
        "target is interpreted at command-time heading");
}

void testVinsQuaternion() {
  const double half = g::kPi / 4.0;
  const g::Mat3 rotation =
      g::quaternionToMat3(0.0, 0.0, std::sin(half), std::cos(half));
  checkVec3(rotation.apply(g::Vec3{1.0, 0.0, 0.0}), 0.0, 1.0, 0.0,
            "quaternion 90 deg yaw", 1e-9);
}

void testVinsTransform() {
  g::Extrinsic extrinsic;
  extrinsic.R_base_cam = g::Mat3::rotationZ(g::kPi / 2.0);
  extrinsic.t_base_cam = g::Vec3{0.5, 0.0, 0.0};

  g::Pose camera;
  camera.position = g::Vec3{0.0, 0.0, 0.0};
  camera.rotation = g::Mat3::identity();
  const g::Pose base = g::basePoseFromCamera(camera, extrinsic);
  checkVec3(base.position, 0.0, 0.5, 0.0, "base pose translation", 1e-9);

  g::Twist camera_twist;
  camera_twist.linear = g::Vec3{0.0, 0.0, 0.0};
  camera_twist.angular = g::Vec3{0.0, 0.0, 0.3};
  const g::Twist base_twist =
      g::baseTwistFromCamera(camera_twist, extrinsic);
  checkVec3(base_twist.linear, 0.0, -0.15, 0.0, "base twist linear", 1e-9);
  checkVec3(base_twist.angular, 0.0, 0.0, 0.3, "base twist angular", 1e-9);
}

g::CameraState makeCameraState(const g::Pose& base,
                               const g::Twist& base_twist,
                               const g::Extrinsic& extrinsic,
                               double stamp_s) {
  g::CameraState camera_state;
  camera_state.stamp_s = stamp_s;
  camera_state.world_camera.rotation =
      base.rotation * extrinsic.R_base_cam;
  camera_state.world_camera.position =
      base.rotation.apply(extrinsic.t_base_cam) + base.position;
  camera_state.body_twist_camera.angular =
      extrinsic.R_base_cam.transposed().apply(base_twist.angular);
  camera_state.body_twist_camera.linear =
      extrinsic.R_base_cam.transposed().apply(
          base_twist.linear +
          g::cross(base_twist.angular, extrinsic.t_base_cam));
  camera_state.valid_input = true;
  return camera_state;
}

void testEndToEndCameraToBase() {
  g::Extrinsic extrinsic;
  extrinsic.R_base_cam.m = {{0.0, 0.0, 1.0,
                             -1.0, 0.0, 0.0,
                             0.0, -1.0, 0.0}};
  extrinsic.t_base_cam = g::Vec3{0.3475, 0.0, 0.0525};
  extrinsic.measured = true;

  const double yaw0 = 20.0 * g::kPi / 180.0;
  const double speed = 0.25;
  const double yaw_rate = 0.2;
  const double dt = 0.1;
  const int steps = 10;
  const g::Vec3 start_position{1.0, 2.0, 0.3};

  std::vector<g::Pose> base_poses;
  std::vector<g::Twist> base_twists;
  for (int i = 0; i <= steps; ++i) {
    const double t = dt * static_cast<double>(i);
    const double yaw = yaw0 + yaw_rate * t;
    g::Pose base;
    base.rotation = g::Mat3::rotationZ(yaw);
    base.position.x = start_position.x +
                      (speed / yaw_rate) * (std::sin(yaw) - std::sin(yaw0));
    base.position.y = start_position.y -
                      (speed / yaw_rate) * (std::cos(yaw) - std::cos(yaw0));
    base.position.z = start_position.z;
    base_poses.push_back(base);

    g::Twist twist;
    twist.linear = g::Vec3{speed, 0.0, 0.0};
    twist.angular = g::Vec3{0.0, 0.0, yaw_rate};
    base_twists.push_back(twist);
  }

  const g::NavFrame nav =
      g::NavFrame::fromInitialBasePose(base_poses.front());
  g::BodyStateEstimator estimator(nav, extrinsic);

  double max_position_error = 0.0;
  double max_yaw_error = 0.0;
  double max_velocity_error = 0.0;

  for (int i = 0; i <= steps; ++i) {
    const g::Pose& base = base_poses[i];
    const g::Twist& base_twist = base_twists[i];

    const g::CameraState camera_state = makeCameraState(
        base, base_twist, extrinsic, dt * static_cast<double>(i));

    const g::BodyState state =
        estimator.update(camera_state, camera_state.stamp_s);
    check(state.valid, "end-to-end state valid");

    const g::Vec3 expected_position = nav.worldToNav(base.position);
    const double expected_yaw =
        g::wrapPi(yaw0 + yaw_rate * camera_state.stamp_s - yaw0);
    checkNear(state.position_m.x, expected_position.x,
              "end-to-end position x", 1e-6);
    checkNear(state.position_m.y, expected_position.y,
              "end-to-end position y", 1e-6);
    checkNear(state.position_m.z, expected_position.z,
              "end-to-end position z", 1e-6);
    checkNear(state.yaw_rad, expected_yaw, "end-to-end yaw", 1e-6);
    checkNear(state.velocity_mps.x, speed, "end-to-end vx", 1e-6);
    checkNear(state.velocity_mps.y, 0.0, "end-to-end vy", 1e-6);
    checkNear(state.yaw_rate_radps, yaw_rate, "end-to-end yaw rate", 1e-6);

    max_position_error =
        std::max(max_position_error,
                 g::norm(state.position_m - expected_position));
    max_yaw_error = std::max(
        max_yaw_error, std::abs(g::wrapPi(state.yaw_rad - expected_yaw)));
    max_velocity_error = std::max(
        max_velocity_error,
        std::hypot(state.velocity_mps.x - speed, state.velocity_mps.y));
  }
  std::cout << "end-to-end camera->base: max position error="
            << max_position_error << " m, max yaw error=" << max_yaw_error
            << " rad, max velocity error=" << max_velocity_error << " m/s\n";
}

void testClosedLoopCameraToBaseToTracker() {
  g::Extrinsic extrinsic;
  extrinsic.R_base_cam.m = {{0.0, 0.0, 1.0,
                             -1.0, 0.0, 0.0,
                             0.0, -1.0, 0.0}};
  extrinsic.t_base_cam = g::Vec3{0.3475, 0.0, 0.0525};
  extrinsic.measured = true;

  const g::Pose2D start_pose{0.0, 0.0, 0.0};
  const g::Pose2D goal_pose{1.0, 0.0, 0.0};
  const g::Path path = g::makeStraightPath(start_pose, goal_pose);

  g::Pose base;
  base.position = g::Vec3{0.0, 0.0, 0.0};
  base.rotation = g::Mat3::identity();
  g::Twist base_twist;

  const g::NavFrame nav = g::NavFrame::fromInitialBasePose(base);
  g::BodyStateEstimator estimator(nav, extrinsic);

  g::PurePursuitConfig config;
  config.lookahead_min_m = 0.20;
  config.lookahead_gain_s = 1.0;
  config.lookahead_max_m = 0.60;
  config.max_speed_mps = 0.30;
  config.min_speed_mps = 0.03;
  config.max_yaw_rate_radps = 0.50;
  config.max_lateral_accel_mps2 = 0.50;
  config.decel_mps2 = 0.30;
  config.stop_distance_m = 0.02;
  config.goal_tolerance_m = 0.02;
  g::PurePursuit tracker(config);
  g::PurePursuitState tracker_state;

  const double dt = 0.01;
  bool stopped = false;
  bool states_valid = true;
  bool any_lateral = false;
  double time_s = 0.0;

  for (int step = 0; step < 6000; ++step) {
    const g::CameraState camera_state =
        makeCameraState(base, base_twist, extrinsic, time_s);
    const g::BodyState state = estimator.update(camera_state, time_s);
    if (!state.valid) {
      states_valid = false;
      break;
    }

    const g::VelocityCommand command =
        tracker.compute(state, path, tracker_state);
    if (command.vy_mps != 0.0) {
      any_lateral = true;
    }
    if (command.stop_request) {
      stopped = true;
      break;
    }

    const double yaw = std::atan2(base.rotation(1, 0), base.rotation(0, 0));
    base.position.x += command.vx_mps * std::cos(yaw) * dt;
    base.position.y += command.vx_mps * std::sin(yaw) * dt;
    base.rotation =
        g::Mat3::rotationZ(g::wrapPi(yaw + command.vyaw_radps * dt));
    base_twist.linear = g::Vec3{command.vx_mps, 0.0, 0.0};
    base_twist.angular = g::Vec3{0.0, 0.0, command.vyaw_radps};
    time_s += dt;
  }

  check(stopped, "closed-loop stopped");
  check(states_valid, "closed-loop states valid");
  check(!any_lateral, "closed-loop commands have vy=0");
  const double final_error =
      std::hypot(base.position.x - goal_pose.x, base.position.y - goal_pose.y);
  check(final_error < 0.05, "closed-loop final error < 5 cm");
  std::cout << "closed-loop camera->base->tracker: final error="
            << final_error << " m, time=" << time_s << " s\n";
}

void testVinsCsvAndReplay() {
  const std::string path = "go2_nav_plan_selftest_input.csv";
  {
    std::ofstream file(path);
    file << "bag_ns,header_s,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,"
            "frame_id,child_frame_id\n";
    file << "1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,"
            "vins_world,camera_imu_optical_frame\n";
    file << "2,0.1,0.1,0.0,0.0,0.0,0.0,0.0,1.0,0.1,0.0,0.0,0.0,0.0,0.0,"
            "vins_world,camera_imu_optical_frame\n";
    file << "3,0.2,0.2,0.0,0.0,0.0,0.0,0.0,1.0,0.1,0.0,0.0,0.0,0.0,0.0,"
            "vins_world,camera_imu_optical_frame\n";
  }

  std::vector<g::VinsOdometryRow> rows;
  std::string error;
  const bool loaded = g::loadVinsOdometryCsv(path, &rows, &error);
  check(loaded, "VINS CSV loaded");
  check(rows.size() == 3, "VINS CSV row count");
  if (loaded && rows.size() == 3) {
    checkNear(rows[1].stamp_s, 0.1, "VINS CSV stamp");
    checkVec3(rows[2].position, 0.2, 0.0, 0.0, "VINS CSV position", 1e-12);
  }

  const g::Extrinsic identity;
  const g::Pose initial_base =
      g::basePoseFromCamera(g::Pose{rows[0].position, rows[0].rotation},
                            identity);
  const g::NavFrame nav = g::NavFrame::fromInitialBasePose(initial_base);
  g::ValidityConfig validity;
  validity.max_age_s = 1.0;
  g::BodyStateEstimator estimator(nav, identity, validity);

  g::BodyState state;
  for (const g::VinsOdometryRow& row : rows) {
    state = estimator.update(g::toCameraState(row), row.stamp_s);
    check(state.valid, "VINS replay state valid");
  }
  checkNear(state.position_m.x, 0.2, "VINS replay final x", 1e-9);
  checkNear(state.position_m.y, 0.0, "VINS replay final y", 1e-9);
  checkNear(state.yaw_rad, 0.0, "VINS replay final yaw", 1e-9);

  std::remove(path.c_str());
}

}  // namespace

int main() {
  testStraightPath();
  testDubinsPath();
  testMakePathStraight();
  testPurePursuitStraight();
  testPurePursuitCurve();
  testTargetSemantics();
  testVinsQuaternion();
  testVinsTransform();
  testEndToEndCameraToBase();
  testClosedLoopCameraToBaseToTracker();
  testVinsCsvAndReplay();

  if (failures != 0) {
    std::cerr << failures << " failure(s)\n";
    return 1;
  }
  std::cout << "all go2_nav_plan selftests passed; no DDS initialized\n";
  return 0;
}
