#include "extrinsic_config.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace go2_nav {
namespace {

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

bool parseBool(const std::string& text, bool* value) {
  if (text == "1" || text == "true" || text == "yes") {
    *value = true;
    return true;
  }
  if (text == "0" || text == "false" || text == "no") {
    *value = false;
    return true;
  }
  return false;
}

}  // namespace

bool loadExtrinsicConfig(const std::string& path, ExtrinsicConfig* config,
                         std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "config output is null";
    }
    return false;
  }
  *config = ExtrinsicConfig{};
  std::ifstream file(path);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "cannot open config: " + path;
    }
    return false;
  }

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    const std::size_t hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    std::istringstream stream(line);
    std::string key;
    std::string value;
    if (!(stream >> key)) {
      continue;
    }
    if (!(stream >> value)) {
      if (error != nullptr) {
        *error = "missing value at line " + std::to_string(line_number);
      }
      return false;
    }

    bool ok = true;
    if (key == "tx_m") {
      ok = parseDouble(value, &config->tx);
    } else if (key == "ty_m") {
      ok = parseDouble(value, &config->ty);
    } else if (key == "tz_m") {
      ok = parseDouble(value, &config->tz);
    } else if (key == "yaw_deg") {
      ok = parseDouble(value, &config->yaw_deg);
    } else if (key == "pitch_deg") {
      ok = parseDouble(value, &config->pitch_deg);
    } else if (key == "roll_deg") {
      ok = parseDouble(value, &config->roll_deg);
    } else if (key == "camera_optical") {
      ok = parseBool(value, &config->camera_optical);
    } else if (key == "measured") {
      ok = parseBool(value, &config->measured);
    } else {
      if (error != nullptr) {
        *error = "unknown key at line " + std::to_string(line_number) + ": " +
                 key;
      }
      return false;
    }
    if (!ok) {
      if (error != nullptr) {
        *error = "invalid value at line " + std::to_string(line_number) +
                 ": " + key + " " + value;
      }
      return false;
    }
  }
  return true;
}

Extrinsic makeExtrinsic(const ExtrinsicConfig& config) {
  Extrinsic extrinsic;
  Mat3 mounting_rotation = Mat3::fromYawPitchRoll(
      config.yaw_deg * kPi / 180.0, config.pitch_deg * kPi / 180.0,
      config.roll_deg * kPi / 180.0);
  if (config.camera_optical) {
    Mat3 optical_to_base;
    optical_to_base.m = {{0.0, 0.0, 1.0,
                          -1.0, 0.0, 0.0,
                          0.0, -1.0, 0.0}};
    mounting_rotation = mounting_rotation * optical_to_base;
  }
  extrinsic.R_base_cam = mounting_rotation;
  extrinsic.t_base_cam = Vec3{config.tx, config.ty, config.tz};
  extrinsic.measured = config.measured;
  return extrinsic;
}

}  // namespace go2_nav
