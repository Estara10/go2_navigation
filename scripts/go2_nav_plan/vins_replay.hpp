#ifndef GO2_NAV_PLAN_VINS_REPLAY_HPP
#define GO2_NAV_PLAN_VINS_REPLAY_HPP

#include <cstddef>
#include <string>
#include <vector>

#include "body_state.hpp"

namespace go2_nav {

// One VINS odometry sample. Position and orientation are the camera IMU pose in
// the VINS world. Linear and angular velocity are the camera IMU body twist,
// expressed in the camera IMU frame.
struct VinsOdometryRow {
  double stamp_s = 0.0;
  Vec3 position;
  Mat3 rotation = Mat3::identity();
  Vec3 linear_velocity;
  Vec3 angular_velocity;
};

Mat3 quaternionToMat3(double qx, double qy, double qz, double qw);

// Reads the CSV produced by the historical VINS review:
// bag_ns,header_s,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id
// Extra columns are ignored. Invalid rows are skipped. Returns false only if
// the file cannot be opened or no valid row remains.
bool loadVinsOdometryCsv(const std::string& path,
                         std::vector<VinsOdometryRow>* rows,
                         std::string* error,
                         std::size_t* skipped_rows = nullptr);

CameraState toCameraState(const VinsOdometryRow& row);

}  // namespace go2_nav

#endif
