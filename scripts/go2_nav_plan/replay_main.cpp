#include "vins_replay.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace g = go2_nav;

namespace {

void printUsage(const char* program) {
  std::cout
      << "用法: " << program << " --csv <vins_odometry.csv> [--out <out.csv>]"
      << " [--config <extrinsic.conf>] [--tx m] [--ty m] [--tz m]"
      << " [--yaw-deg deg] [--pitch-deg deg] [--roll-deg deg]"
      << " [--camera-optical] [--measured]\n"
      << "把 VINS 相机 IMU 里程计按 T_base_cam 转成 BodyState。\n"
      << "--config 先读外参文件，后面的命令行参数覆盖文件值。\n"
      << "--camera-optical 使用相机光学系到 base_link 的标称旋转"
         "（X右/Y下/Z前 → X前/Y左/Z上）；yaw/pitch/roll 再绕 base_link 旋转。\n"
      << "默认外参为单位变换且 measured=false；默认输出是相机 IMU 参考点，"
         "不是实测机身参考点。\n"
      << "只读 CSV，不初始化 DDS，不连接机器人或相机。\n";
}

bool parseDouble(const char* text, double* value) {
  try {
    std::size_t position = 0;
    const double parsed = std::stod(text, &position);
    if (position != std::string(text).size() || !std::isfinite(parsed)) {
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
                         std::string* error) {
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
      ok = parseDouble(value.c_str(), &config->tx);
    } else if (key == "ty_m") {
      ok = parseDouble(value.c_str(), &config->ty);
    } else if (key == "tz_m") {
      ok = parseDouble(value.c_str(), &config->tz);
    } else if (key == "yaw_deg") {
      ok = parseDouble(value.c_str(), &config->yaw_deg);
    } else if (key == "pitch_deg") {
      ok = parseDouble(value.c_str(), &config->pitch_deg);
    } else if (key == "roll_deg") {
      ok = parseDouble(value.c_str(), &config->roll_deg);
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

}  // namespace

int main(int argc, char** argv) {
  std::string csv_path;
  std::string out_path;
  std::string config_path;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config" && i + 1 < argc) {
      config_path = argv[i + 1];
    }
  }

  ExtrinsicConfig config;
  if (!config_path.empty()) {
    std::string error;
    if (!loadExtrinsicConfig(config_path, &config, &error)) {
      std::cerr << "failed to load config: " << error << "\n";
      return 2;
    }
  }

  double tx = config.tx;
  double ty = config.ty;
  double tz = config.tz;
  double yaw_deg = config.yaw_deg;
  double pitch_deg = config.pitch_deg;
  double roll_deg = config.roll_deg;
  bool camera_optical = config.camera_optical;
  bool measured = config.measured;
  bool help = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      help = true;
    } else if (arg == "--csv" && i + 1 < argc) {
      csv_path = argv[++i];
    } else if (arg == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else if (arg == "--tx" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &tx)) {
        std::cerr << "invalid --tx\n";
        return 2;
      }
    } else if (arg == "--ty" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &ty)) {
        std::cerr << "invalid --ty\n";
        return 2;
      }
    } else if (arg == "--tz" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &tz)) {
        std::cerr << "invalid --tz\n";
        return 2;
      }
    } else if (arg == "--yaw-deg" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &yaw_deg)) {
        std::cerr << "invalid --yaw-deg\n";
        return 2;
      }
    } else if (arg == "--camera-optical") {
      camera_optical = true;
    } else if (arg == "--pitch-deg" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &pitch_deg)) {
        std::cerr << "invalid --pitch-deg\n";
        return 2;
      }
    } else if (arg == "--roll-deg" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &roll_deg)) {
        std::cerr << "invalid --roll-deg\n";
        return 2;
      }
    } else if (arg == "--measured") {
      measured = true;
    } else if (arg == "--config" && i + 1 < argc) {
      ++i;  // already loaded in the pre-scan
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 2;
    }
  }

  if (help) {
    printUsage(argv[0]);
    return 0;
  }
  if (csv_path.empty()) {
    std::cerr << "--csv is required\n";
    printUsage(argv[0]);
    return 2;
  }

  std::vector<g::VinsOdometryRow> rows;
  std::string load_error;
  std::size_t skipped_rows = 0;
  if (!g::loadVinsOdometryCsv(csv_path, &rows, &load_error, &skipped_rows)) {
    std::cerr << "failed to load CSV: " << load_error << "\n";
    return 1;
  }
  if (skipped_rows > 0) {
    std::cerr << "warning: " << skipped_rows << " rows skipped\n";
  }

  g::Extrinsic extrinsic;
  g::Mat3 mounting_rotation = g::Mat3::fromYawPitchRoll(
      yaw_deg * g::kPi / 180.0, pitch_deg * g::kPi / 180.0,
      roll_deg * g::kPi / 180.0);
  if (camera_optical) {
    g::Mat3 optical_to_base;
    optical_to_base.m = {{0.0, 0.0, 1.0,
                          -1.0, 0.0, 0.0,
                          0.0, -1.0, 0.0}};
    mounting_rotation = mounting_rotation * optical_to_base;
  }
  extrinsic.R_base_cam = mounting_rotation;
  extrinsic.t_base_cam = g::Vec3{tx, ty, tz};
  extrinsic.measured = measured;

  const g::Pose initial_camera{rows.front().position, rows.front().rotation};
  const g::Pose initial_base =
      g::basePoseFromCamera(initial_camera, extrinsic);
  const g::NavFrame nav = g::NavFrame::fromInitialBasePose(initial_base);

  // Smoke-test validity: the historical CSV may contain large re-anchor jumps.
  // Production thresholds are separate and must not be inferred from this run.
  g::ValidityConfig validity;
  validity.max_age_s = 1e9;
  validity.max_future_skew_s = 1e9;
  validity.max_position_jump_m = 1e9;
  validity.max_yaw_jump_rad = g::kPi;

  g::BodyStateEstimator estimator(nav, extrinsic, validity);

  std::ofstream out;
  if (!out_path.empty()) {
    out.open(out_path);
    if (!out.is_open()) {
      std::cerr << "cannot open output: " << out_path << "\n";
      return 2;
    }
    out << "stamp_s,x_m,y_m,yaw_rad,vx_mps,vy_mps,yaw_rate_radps,valid\n";
    out << std::setprecision(12);
  }

  std::size_t valid_count = 0;
  g::BodyState first_state;
  g::BodyState last_state;
  bool have_first = false;
  bool have_previous = false;
  g::Vec3 previous_position;
  double max_jump_m = 0.0;
  double max_speed_mps = 0.0;

  for (const g::VinsOdometryRow& row : rows) {
    const g::BodyState state =
        estimator.update(g::toCameraState(row), row.stamp_s);
    if (state.valid) {
      ++valid_count;
      if (!have_first) {
        first_state = state;
        have_first = true;
      }
      last_state = state;
      if (have_previous) {
        max_jump_m =
            std::max(max_jump_m,
                     g::norm(state.position_m - previous_position));
      }
      previous_position = state.position_m;
      have_previous = true;
      max_speed_mps =
          std::max(max_speed_mps,
                   std::hypot(state.velocity_mps.x, state.velocity_mps.y));
    }
    if (out.is_open()) {
      out << state.stamp_s << ',' << state.position_m.x << ','
          << state.position_m.y << ',' << state.yaw_rad << ','
          << state.velocity_mps.x << ',' << state.velocity_mps.y << ','
          << state.yaw_rate_radps << ',' << (state.valid ? 1 : 0) << '\n';
    }
  }

  std::cout << std::setprecision(6);
  std::cout << "rows: " << rows.size() << "\n";
  std::cout << "valid: " << valid_count << "\n";
  if (have_first) {
    std::cout << "first nav: x=" << first_state.position_m.x
              << " y=" << first_state.position_m.y
              << " yaw=" << first_state.yaw_rad << "\n";
    std::cout << "last nav:  x=" << last_state.position_m.x
              << " y=" << last_state.position_m.y
              << " yaw=" << last_state.yaw_rad << "\n";
    std::cout << "max adjacent jump: " << max_jump_m << " m\n";
    std::cout << "max body speed: " << max_speed_mps << " m/s\n";
  }
  std::cout << "extrinsic: tx=" << tx << " ty=" << ty << " tz=" << tz
            << " yaw_deg=" << yaw_deg << " pitch_deg=" << pitch_deg
            << " roll_deg=" << roll_deg
            << " camera_optical=" << (camera_optical ? "yes" : "no")
            << " measured=" << (measured ? "true" : "false") << "\n";
  std::cout << "note: default/identity extrinsic output is the camera IMU "
               "reference point, not the measured base_link point.\n";

  return valid_count > 0 ? 0 : 1;
}
