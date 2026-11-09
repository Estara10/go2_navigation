#include "vins_replay.hpp"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace go2_nav {
namespace {

std::string trim(const std::string& text) {
  const std::size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::string();
  }
  const std::size_t last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= line.size()) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(trim(line.substr(start)));
      break;
    }
    fields.push_back(trim(line.substr(start, comma - start)));
    start = comma + 1;
  }
  return fields;
}

bool parseDouble(const std::string& text, double* value) {
  try {
    std::size_t position = 0;
    const double parsed = std::stod(text, &position);
    if (position != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

Mat3 quaternionToMat3(double qx, double qy, double qz, double qw) {
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (!(norm > 1e-12) || !std::isfinite(norm)) {
    return Mat3::identity();
  }
  qx /= norm;
  qy /= norm;
  qz /= norm;
  qw /= norm;

  const double xx = qx * qx;
  const double yy = qy * qy;
  const double zz = qz * qz;
  const double xy = qx * qy;
  const double xz = qx * qz;
  const double yz = qy * qz;
  const double wx = qw * qx;
  const double wy = qw * qy;
  const double wz = qw * qz;

  Mat3 rotation;
  rotation.m = {{1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
                 2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
                 2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)}};
  return rotation;
}

bool loadVinsOdometryCsv(const std::string& path,
                         std::vector<VinsOdometryRow>* rows,
                         std::string* error,
                         std::size_t* skipped_rows) {
  if (rows == nullptr) {
    if (error != nullptr) {
      *error = "rows is null";
    }
    return false;
  }
  rows->clear();

  std::ifstream file(path);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "cannot open CSV: " + path;
    }
    return false;
  }

  std::string line;
  bool first_data_line = true;
  std::size_t skipped = 0;
  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    if (first_data_line) {
      first_data_line = false;
      if (line.find("bag_ns") != std::string::npos ||
          line.find("header_s") != std::string::npos) {
        continue;
      }
    }

    const std::vector<std::string> fields = splitCsv(line);
    if (fields.size() < 15) {
      ++skipped;
      continue;
    }

    double stamp = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
    const bool ok =
        parseDouble(fields[1], &stamp) && parseDouble(fields[2], &x) &&
        parseDouble(fields[3], &y) && parseDouble(fields[4], &z) &&
        parseDouble(fields[5], &qx) && parseDouble(fields[6], &qy) &&
        parseDouble(fields[7], &qz) && parseDouble(fields[8], &qw) &&
        parseDouble(fields[9], &vx) && parseDouble(fields[10], &vy) &&
        parseDouble(fields[11], &vz) && parseDouble(fields[12], &wx) &&
        parseDouble(fields[13], &wy) && parseDouble(fields[14], &wz);
    if (!ok) {
      ++skipped;
      continue;
    }

    VinsOdometryRow row;
    row.stamp_s = stamp;
    row.position = Vec3{x, y, z};
    row.rotation = quaternionToMat3(qx, qy, qz, qw);
    row.linear_velocity = Vec3{vx, vy, vz};
    row.angular_velocity = Vec3{wx, wy, wz};
    rows->push_back(row);
  }

  if (rows->empty()) {
    if (error != nullptr) {
      *error = "no valid rows in CSV: " + path;
    }
    return false;
  }
  if (skipped_rows != nullptr) {
    *skipped_rows = skipped;
  }
  return true;
}

CameraState toCameraState(const VinsOdometryRow& row) {
  CameraState camera;
  camera.stamp_s = row.stamp_s;
  camera.world_camera.position = row.position;
  camera.world_camera.rotation = row.rotation;
  camera.body_twist_camera.linear = row.linear_velocity;
  camera.body_twist_camera.angular = row.angular_velocity;
  camera.valid_input = true;
  return camera;
}

}  // namespace go2_nav
