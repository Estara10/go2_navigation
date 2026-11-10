#include "bridge_core.hpp"
#include "udp_bridge_server.hpp"

#include "body_state.hpp"
#include "extrinsic_config.hpp"
#include "vins_replay.hpp"
#include "vins_wire.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

double monotonicNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void printUsage(const char* program) {
  std::cout
      << "用法: " << program
      << " --config <extrinsic.conf> [--control-port 46001]"
         " [--state-port 46002]\n"
      << "       [--vins-topic /odometry] [--imu-topic <topic>]"
         " [--vins-timeout-s 0.5]\n"
      << "       [--jump-position-m 0.25] [--jump-yaw-deg 30]"
         " [--rate-hz 20]\n"
      << "读取 T_base_cam 配置，订阅 VINS 里程计，转换为 base_link BodyState，"
         "通过 UDP 发给原生工具。\n"
      << "只发状态和 ORIGIN 响应，不发运动命令；不初始化 DDS 以外的设备。\n"
      << "控制端口接收 SET_ORIGIN/PING，状态端口发送 ORIGIN_OK/FAIL 和 STATE。\n";
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

bool parsePort(const std::string& text, uint16_t* value) {
  try {
    std::size_t position = 0;
    const long parsed = std::stol(text, &position);
    if (position != text.size() || parsed < 1 || parsed > 65535) {
      return false;
    }
    *value = static_cast<uint16_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

go2_nav::CameraState cameraFromOdometry(
    const nav_msgs::msg::Odometry& message, double receive_stamp_s) {
  go2_nav::CameraState camera;
  camera.stamp_s = receive_stamp_s;
  camera.world_camera.position = go2_nav::Vec3{
      message.pose.pose.position.x, message.pose.pose.position.y,
      message.pose.pose.position.z};
  const double qx = message.pose.pose.orientation.x;
  const double qy = message.pose.pose.orientation.y;
  const double qz = message.pose.pose.orientation.z;
  const double qw = message.pose.pose.orientation.w;
  const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  camera.world_camera.rotation =
      (std::isfinite(qnorm) && qnorm > 1e-6)
          ? go2_nav::quaternionToMat3(qx, qy, qz, qw)
          : go2_nav::Mat3{};
  camera.body_twist_camera.linear = go2_nav::Vec3{
      message.twist.twist.linear.x, message.twist.twist.linear.y,
      message.twist.twist.linear.z};
  camera.body_twist_camera.angular = go2_nav::Vec3{
      message.twist.twist.angular.x, message.twist.twist.angular.y,
      message.twist.twist.angular.z};
  camera.valid_input =
      std::isfinite(qnorm) && qnorm > 1e-6;
  return camera;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  std::string vins_topic = "/odometry";
  std::string imu_topic;
  uint16_t control_port = 46001;
  uint16_t state_port = 46002;
  double vins_timeout_s = 0.5;
  double jump_position_m = 0.25;
  double jump_yaw_deg = 30.0;
  double rate_hz = 20.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take = [&](std::string* value) {
      if (i + 1 >= argc) {
        return false;
      }
      *value = argv[++i];
      return true;
    };
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--config") {
      if (!take(&config_path)) {
        std::cerr << "--config requires a value\n";
        return 2;
      }
    } else if (arg == "--control-port") {
      std::string value;
      if (!take(&value) || !parsePort(value, &control_port)) {
        std::cerr << "invalid --control-port\n";
        return 2;
      }
    } else if (arg == "--state-port") {
      std::string value;
      if (!take(&value) || !parsePort(value, &state_port)) {
        std::cerr << "invalid --state-port\n";
        return 2;
      }
    } else if (arg == "--vins-topic") {
      if (!take(&vins_topic)) {
        std::cerr << "--vins-topic requires a value\n";
        return 2;
      }
    } else if (arg == "--imu-topic") {
      if (!take(&imu_topic)) {
        std::cerr << "--imu-topic requires a value\n";
        return 2;
      }
    } else if (arg == "--vins-timeout-s") {
      std::string value;
      if (!take(&value) || !parseDouble(value, &vins_timeout_s) ||
          vins_timeout_s <= 0.0 || vins_timeout_s > 5.0) {
        std::cerr << "invalid --vins-timeout-s\n";
        return 2;
      }
    } else if (arg == "--jump-position-m") {
      std::string value;
      if (!take(&value) || !parseDouble(value, &jump_position_m) ||
          jump_position_m <= 0.0) {
        std::cerr << "invalid --jump-position-m\n";
        return 2;
      }
    } else if (arg == "--jump-yaw-deg") {
      std::string value;
      if (!take(&value) || !parseDouble(value, &jump_yaw_deg) ||
          jump_yaw_deg <= 0.0 || jump_yaw_deg > 180.0) {
        std::cerr << "invalid --jump-yaw-deg\n";
        return 2;
      }
    } else if (arg == "--rate-hz") {
      std::string value;
      if (!take(&value) || !parseDouble(value, &rate_hz) || rate_hz <= 0.0 ||
          rate_hz > 200.0) {
        std::cerr << "invalid --rate-hz\n";
        return 2;
      }
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 2;
    }
  }

  if (config_path.empty()) {
    std::cerr << "--config is required\n";
    printUsage(argv[0]);
    return 2;
  }
  if (control_port == state_port) {
    std::cerr << "--control-port and --state-port must differ\n";
    return 2;
  }

  go2_nav::ExtrinsicConfig extrinsic_config;
  std::string error;
  if (!go2_nav::loadExtrinsicConfig(config_path, &extrinsic_config, &error)) {
    std::cerr << "failed to load config: " << error << "\n";
    return 2;
  }

  go2_vins_bridge::BridgeConfig bridge_config;
  bridge_config.extrinsic = go2_nav::makeExtrinsic(extrinsic_config);
  bridge_config.vins_timeout_s = vins_timeout_s;
  bridge_config.jump_position_m = jump_position_m;
  bridge_config.jump_yaw_rad = jump_yaw_deg * go2_nav::kPi / 180.0;

  go2_vins_bridge::BridgeCore core(bridge_config);
  go2_vins_bridge::UdpBridgeServer server(control_port, state_port);
  if (!server.open(&error)) {
    std::cerr << "failed to open UDP bridge: " << error << "\n";
    return 2;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go2_vins_bridge");
  if (!bridge_config.extrinsic.measured) {
    RCLCPP_WARN(node->get_logger(),
                "T_base_cam measured=false (provisional); "
                "do not use this setup for 5 cm acceptance");
  }

  auto odom_callback = [&](const nav_msgs::msg::Odometry::SharedPtr message) {
    const double receive_stamp_s = monotonicNow();
    go2_vins_bridge::VinsSample sample;
    sample.header_stamp_s = static_cast<double>(message->header.stamp.sec) +
                            static_cast<double>(message->header.stamp.nanosec) *
                                1e-9;
    sample.receive_stamp_s = receive_stamp_s;
    sample.camera = cameraFromOdometry(*message, receive_stamp_s);
    core.onVinsSample(sample, receive_stamp_s);
  };

  auto odom_subscription = node->create_subscription<nav_msgs::msg::Odometry>(
      vins_topic, rclcpp::SensorDataQoS(), odom_callback);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_subscription;
  if (!imu_topic.empty() && imu_topic != vins_topic) {
    imu_subscription = node->create_subscription<nav_msgs::msg::Odometry>(
        imu_topic, rclcpp::SensorDataQoS(), odom_callback);
  }

  auto handler = [&](const std::string& text) -> std::string {
    bool set_origin = false;
    bool ping = false;
    std::string decode_error;
    if (!go2_nav::decodeNativeCommand(text, &set_origin, &ping,
                                      &decode_error)) {
      return std::string();
    }
    if (set_origin) {
      go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
      if (core.setOrigin(monotonicNow(), &reason)) {
        return go2_nav::encodeOriginOk(core.extrinsic().measured);
      }
      return go2_nav::encodeOriginFail(reason);
    }
    if (ping) {
      return go2_nav::encodeState(core.makeState(monotonicNow()));
    }
    return std::string();
  };

  std::atomic<bool> stop{false};
  std::thread udp_thread([&]() {
    while (!stop.load() && rclcpp::ok()) {
      std::string poll_error;
      if (!server.pollOnce(100, handler, &poll_error)) {
        RCLCPP_ERROR(node->get_logger(), "UDP control error: %s",
                     poll_error.c_str());
        break;
      }
    }
  });

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  auto timer = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [&]() {
        const double now_s = monotonicNow();
        const go2_nav::VinsWireState state = core.makeState(now_s);
        std::string send_error;
        if (!server.sendState(go2_nav::encodeState(state), &send_error)) {
          RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               "UDP state send failed: %s",
                               send_error.c_str());
        }
        const go2_vins_bridge::Diagnostics diagnostics = core.diagnostics();
        RCLCPP_INFO_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "nav=%d valid=%d reason=%s header=%.6f receive=%.6f "
            "measured=%d",
            diagnostics.has_nav ? 1 : 0, state.valid ? 1 : 0,
            go2_nav::vinsReasonName(state.reason),
            diagnostics.header_stamp_s, diagnostics.receive_stamp_s,
            bridge_config.extrinsic.measured ? 1 : 0);
      });

  RCLCPP_INFO(node->get_logger(),
              "VINS bridge: topic=%s imu_topic=%s control_port=%u "
              "state_port=%u config=%s",
              vins_topic.c_str(),
              imu_topic.empty() ? "(none)" : imu_topic.c_str(),
              static_cast<unsigned>(control_port),
              static_cast<unsigned>(state_port), config_path.c_str());

  try {
    rclcpp::spin(node);
  } catch (const std::exception& spin_error) {
    RCLCPP_ERROR(node->get_logger(), "ROS spin failed: %s",
                 spin_error.what());
  }
  stop.store(true);
  if (udp_thread.joinable()) {
    udp_thread.join();
  }
  server.close();
  rclcpp::shutdown();
  return 0;
}
