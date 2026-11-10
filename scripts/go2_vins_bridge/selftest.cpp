#include "bridge_core.hpp"
#include "udp_bridge_server.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "extrinsic_config.hpp"
#include "vins_wire.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& name) {
  if (!condition) {
    std::cerr << "FAIL " << name << "\n";
    ++failures;
  }
}

double monotonicNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

go2_nav::CameraState makeCamera(double x, double y, double yaw, double stamp) {
  go2_nav::CameraState camera;
  camera.stamp_s = stamp;
  camera.world_camera.position = go2_nav::Vec3{x, y, 0.0};
  camera.world_camera.rotation = go2_nav::Mat3::rotationZ(yaw);
  camera.valid_input = true;
  return camera;
}

go2_vins_bridge::VinsSample makeSample(double header, double receive, double x,
                                       double y, double yaw) {
  go2_vins_bridge::VinsSample sample;
  sample.header_stamp_s = header;
  sample.receive_stamp_s = receive;
  sample.camera = makeCamera(x, y, yaw, receive);
  return sample;
}

go2_vins_bridge::BridgeConfig makeConfig() {
  go2_vins_bridge::BridgeConfig config;
  config.vins_timeout_s = 0.5;
  config.jump_position_m = 0.25;
  config.jump_yaw_rad = 30.0 * go2_nav::kPi / 180.0;
  return config;
}

int recvWithTimeout(int socket_fd, std::string* text) {
  struct pollfd descriptor {};
  descriptor.fd = socket_fd;
  descriptor.events = POLLIN;
  const int ready = ::poll(&descriptor, 1, 1000);
  if (ready <= 0) {
    return -1;
  }
  char buffer[512];
  const ssize_t count = ::recvfrom(socket_fd, buffer, sizeof(buffer), 0,
                                   nullptr, nullptr);
  if (count <= 0) {
    return -1;
  }
  *text = std::string(buffer, static_cast<std::size_t>(count));
  return 0;
}

void testExtrinsicConfig() {
  const std::string path = "/tmp/go2_vins_bridge_extrinsic_test.conf";
  {
    std::ofstream file(path);
    file << "tx_m 0.3475\n"
            "ty_m 0.0\n"
            "tz_m 0.0525\n"
            "yaw_deg 0.0\n"
            "pitch_deg 0.0\n"
            "roll_deg 0.0\n"
            "camera_optical 1\n"
            "measured 0\n";
  }
  go2_nav::ExtrinsicConfig config;
  std::string error;
  check(go2_nav::loadExtrinsicConfig(path, &config, &error),
        "extrinsic config load: " + error);
  check(!config.measured, "extrinsic config measured false");
  check(config.camera_optical, "extrinsic config camera_optical true");
  const go2_nav::Extrinsic extrinsic = go2_nav::makeExtrinsic(config);
  check(!extrinsic.measured, "extrinsic built measured false");
  check(std::abs(extrinsic.t_base_cam.x - 0.3475) < 1e-12,
        "extrinsic tx");
  const go2_nav::Vec3 camera_x =
      extrinsic.R_base_cam.apply(go2_nav::Vec3{1.0, 0.0, 0.0});
  check(std::abs(camera_x.x) < 1e-12 && std::abs(camera_x.y + 1.0) < 1e-12 &&
            std::abs(camera_x.z) < 1e-12,
        "extrinsic optical rotation");
  config.pitch_deg = -7.0;
  const auto upward = go2_nav::makeExtrinsic(config);
  const auto optical_forward =
      upward.R_base_cam.apply(go2_nav::Vec3{0.0, 0.0, 1.0});
  check(optical_forward.z > 0.0,
        "negative pitch points optical forward axis upward");
  std::remove(path.c_str());
}

void testCoreExtrinsicRotation() {
  go2_vins_bridge::BridgeConfig config = makeConfig();
  config.extrinsic.R_base_cam = go2_nav::Mat3::rotationZ(go2_nav::kPi / 2.0);
  go2_vins_bridge::BridgeCore core(config);

  core.onVinsSample(makeSample(1.0, 0.0, 0.0, 0.0, go2_nav::kPi / 2.0), 0.0);
  go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
  check(core.setOrigin(0.0, &reason), "bridge rotated extrinsic origin");
  go2_nav::VinsWireState state = core.makeState(0.0);
  check(state.valid && std::abs(state.yaw) < 1e-9,
        "bridge rotated extrinsic base yaw zero");

  core.onVinsSample(
      makeSample(1.1, 0.1, 0.0, 0.0, 100.0 * go2_nav::kPi / 180.0), 0.1);
  state = core.makeState(0.1);
  check(state.valid, "bridge rotated extrinsic moved valid");
  check(std::abs(state.yaw - 10.0 * go2_nav::kPi / 180.0) < 1e-9,
        "bridge rotated extrinsic base yaw relative");
}

void testCoreNoNav() {
  go2_vins_bridge::BridgeCore core(makeConfig());
  go2_nav::VinsWireState state = core.makeState(0.0);
  check(!state.valid &&
            state.reason == go2_nav::VinsReason::NoVins,
        "bridge no sample NO_VINS");

  const auto sample = makeSample(1.0, 0.0, 0.0, 0.0, 0.0);
  core.onVinsSample(sample, 0.0);
  state = core.makeState(0.0);
  check(!state.valid &&
            state.reason == go2_nav::VinsReason::NoNav,
        "bridge sample before origin NO_NAV");
}

void testCoreOriginAndState() {
  go2_vins_bridge::BridgeConfig config = makeConfig();
  config.jump_position_m = 2.0;
  go2_vins_bridge::BridgeCore core(config);
  const auto sample = makeSample(1.0, 0.0, 0.0, 0.0, 0.0);
  core.onVinsSample(sample, 0.0);

  go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
  check(core.setOrigin(0.0, &reason), "bridge origin succeeds");
  check(reason == go2_nav::VinsReason::Ok, "bridge origin reason OK");
  check(core.hasNav(), "bridge has nav");

  go2_nav::VinsWireState state = core.makeState(0.0);
  check(state.valid && state.reason == go2_nav::VinsReason::Ok,
        "bridge origin state valid");
  check(!state.measured, "bridge origin measured false");
  check(std::abs(state.x) < 1e-12 && std::abs(state.y) < 1e-12,
        "bridge origin x/y zero");

  go2_vins_bridge::BridgeConfig measured_config = makeConfig();
  measured_config.extrinsic.measured = true;
  go2_vins_bridge::BridgeCore measured_core(measured_config);
  measured_core.onVinsSample(sample, 0.0);
  go2_nav::VinsReason measured_reason = go2_nav::VinsReason::NoVins;
  check(measured_core.setOrigin(0.0, &measured_reason),
        "bridge measured origin succeeds");
  check(measured_core.makeState(0.0).measured,
        "bridge measured state true");

  auto moved = makeSample(1.1, 0.1, 1.0, 0.0, 0.2);
  moved.camera.body_twist_camera.linear = go2_nav::Vec3{0.2, 0.1, 0.0};
  moved.camera.body_twist_camera.angular = go2_nav::Vec3{0.0, 0.0, 0.3};
  core.onVinsSample(moved, 0.1);
  state = core.makeState(0.1);
  check(state.valid, "bridge moved state valid");
  check(std::abs(state.x - 1.0) < 1e-9, "bridge moved x");
  check(std::abs(state.y) < 1e-9, "bridge moved y");
  check(std::abs(state.yaw - 0.2) < 1e-9, "bridge moved yaw");
  check(std::abs(state.vx - 0.2) < 1e-9, "bridge moved vx");
  check(std::abs(state.vy - 0.1) < 1e-9, "bridge moved vy");
  check(std::abs(state.wz - 0.3) < 1e-9, "bridge moved wz");
}

void testCoreStaleJumpReset() {
  go2_vins_bridge::BridgeCore core(makeConfig());
  core.onVinsSample(makeSample(1.0, 0.0, 0.0, 0.0, 0.0), 0.0);
  go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
  check(core.setOrigin(0.0, &reason), "bridge stale test origin");

  go2_nav::VinsWireState state = core.makeState(1.0);
  check(!state.valid && state.reason == go2_nav::VinsReason::Stale,
        "bridge timeout STALE");

  core.onVinsSample(makeSample(1.1, 1.0, 0.1, 0.0, 0.0), 1.0);
  for (int i = 1; i <= 11; ++i) {
    const double t = 1.0 + 0.1 * i;
    core.onVinsSample(makeSample(1.1 + 0.1 * i, t, 0.1, 0.0, 0.0), t);
  }
  state = core.makeState(2.1);
  check(state.valid && std::abs(state.x - 0.1) < 1e-9,
        "bridge confirms recovery after stale");

  core.onVinsSample(makeSample(2.3, 2.2, 3.0, 0.0, 0.0), 2.2);
  state = core.makeState(2.2);
  check(!state.valid && state.reason == go2_nav::VinsReason::Jump,
        "bridge position jump JUMP");

  core.onVinsSample(makeSample(1.0, 2.3, 3.0, 0.0, 0.0), 2.3);
  state = core.makeState(2.3);
  check(!state.valid && state.reason == go2_nav::VinsReason::Reset,
        "bridge header reset RESET");
  reason = go2_nav::VinsReason::Ok;
  check(!core.setOrigin(2.3, &reason), "bridge origin after reset fails");
  check(reason == go2_nav::VinsReason::Reset,
        "bridge origin after reset reason RESET");
}

void testHealthHeaderStallAndRecovery() {
  go2_vins_bridge::BridgeConfig config = makeConfig();
  config.header_stall_s = 0.25;
  config.recovery_confirm_s = 0.6;
  go2_vins_bridge::BridgeCore core(config);
  core.onVinsSample(makeSample(1.0, 0.0, 0.0, 0.0, 0.0), 0.0);
  check(core.diagnostics().health ==
            go2_vins_bridge::HealthState::Uninitialized,
        "health starts uninitialized before nav origin");
  go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
  check(core.setOrigin(0.0, &reason), "health header test origin");
  check(core.diagnostics().health == go2_vins_bridge::HealthState::Healthy,
        "origin establishes healthy state");
  auto state = core.makeState(0.3);
  check(!state.valid && state.reason == go2_nav::VinsReason::Stale,
        "monotonic freshness cannot hide stalled source header");
  check(core.diagnostics().health == go2_vins_bridge::HealthState::Failed,
        "header stall enters failed state");

  for (int i = 1; i <= 8; ++i) {
    const double t = 0.1 * i;
    core.onVinsSample(makeSample(1.0 + t, t, 0.0, 0.0, 0.0), t);
  }
  state = core.makeState(0.8);
  check(state.valid, "header progression confirms recovery");
  check(core.diagnostics().health ==
            go2_vins_bridge::HealthState::RecoveryConfirmed,
        "health recovery confirmation state");
}

void testHealthLowFeaturesAndSlowDrift() {
  {
    go2_vins_bridge::BridgeConfig config = makeConfig();
    config.recovery_confirm_s = 0.4;
    go2_vins_bridge::BridgeCore core(config);
    core.onVinsSample(makeSample(1.0, 0.0, 0.0, 0.0, 0.0), 0.0);
    go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
    check(core.setOrigin(0.0, &reason), "feature test origin");
    core.onFeatureCount(23, 1.0, 0.0);
    auto state = core.makeState(0.0);
    check(!state.valid && state.reason == go2_nav::VinsReason::Jump,
          "low feature count invalidates VINS state");
    reason = go2_nav::VinsReason::Ok;
    check(!core.setOrigin(0.0, &reason) && reason == go2_nav::VinsReason::Jump,
          "low feature count cannot establish a new origin");
    for (int i = 1; i <= 6; ++i) {
      const double t = 0.1 * i;
      core.onFeatureCount(45, 1.0 + t, t);
      core.onVinsSample(makeSample(1.0 + t, t, 0.0, 0.0, 0.0), t);
    }
    state = core.makeState(0.6);
    check(state.valid && core.diagnostics().health ==
                             go2_vins_bridge::HealthState::RecoveryConfirmed,
          "feature recovery confirms without requiring old task resume");
  }
  {
    go2_vins_bridge::BridgeConfig config = makeConfig();
    go2_vins_bridge::BridgeCore core(config);
    core.onVinsSample(makeSample(1.0, 0.0, 0.0, 0.0, 0.0), 0.0);
    go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
    check(core.setOrigin(0.0, &reason), "feature header stall origin");
    core.onFeatureCount(45, 5.0, 0.0);
    for (int i = 1; i <= 6; ++i) {
      const double t = 0.1 * i;
      core.onFeatureCount(45, 5.0, t);  // receive time advances, source does not
      core.onVinsSample(makeSample(1.0 + t, t, 0.0, 0.0, 0.0), t);
    }
    const auto state = core.makeState(0.6);
    check(!state.valid && state.reason == go2_nav::VinsReason::Jump,
          "fresh feature callbacks with stalled source header are rejected");
  }
  {
    go2_vins_bridge::BridgeConfig config = makeConfig();
    config.slow_drift_window_s = 1.0;
    config.slow_drift_error_m = 0.15;
    config.jump_position_m = 0.25;
    go2_vins_bridge::BridgeCore core(config);
    core.onVinsSample(makeSample(1.0, 0.0, 0.0, 0.0, 0.0), 0.0);
    go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
    check(core.setOrigin(0.0, &reason), "slow drift test origin");
    for (int i = 1; i <= 10; ++i) {
      const double t = 0.1 * i;
      core.onVinsSample(makeSample(1.0 + t, t, 0.02 * i, 0.0, 0.0), t);
    }
    auto state = core.makeState(1.0);
    check(!state.valid && core.diagnostics().health ==
                              go2_vins_bridge::HealthState::Failed,
          "slow pose drift without matching twist invalidates state");
    for (int i = 11; i <= 22; ++i) {
      const double t = 0.1 * i;
      core.onVinsSample(makeSample(1.0 + t, t, 0.20, 0.0, 0.0), t);
    }
    state = core.makeState(2.2);
    check(state.valid && core.diagnostics().health ==
                             go2_vins_bridge::HealthState::RecoveryConfirmed,
          "slow drift recovery requires a clean confirmation window");
  }
}

void testCoreNonFinite() {
  go2_vins_bridge::BridgeCore core(makeConfig());
  core.onVinsSample(makeSample(1.0, 0.0, 0.0, 0.0, 0.0), 0.0);
  go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
  check(core.setOrigin(0.0, &reason), "bridge nonfinite test origin");

  auto bad = makeSample(1.1, 0.1, 1.0, 0.0, 0.0);
  bad.camera.world_camera.position.x =
      std::numeric_limits<double>::quiet_NaN();
  core.onVinsSample(bad, 0.1);
  const go2_nav::VinsWireState state = core.makeState(0.1);
  check(!state.valid && state.reason == go2_nav::VinsReason::NonFinite,
        "bridge nonfinite NONFINITE");
}

void testUdpLoopback() {
  constexpr uint16_t kControlPort = 47201;
  constexpr uint16_t kStatePort = 47202;
  go2_vins_bridge::BridgeCore core(makeConfig());
  core.onVinsSample(makeSample(1.0, monotonicNow(), 0.0, 0.0, 0.0),
                    monotonicNow());
  go2_nav::VinsReason reason = go2_nav::VinsReason::NoVins;
  check(core.setOrigin(monotonicNow(), &reason), "bridge udp origin");

  go2_vins_bridge::UdpBridgeServer server(kControlPort, kStatePort);
  std::string error;
  check(server.open(&error), "bridge udp server open: " + error);

  const int client = ::socket(AF_INET, SOCK_DGRAM, 0);
  check(client >= 0, "bridge udp client socket");
  struct sockaddr_in local {};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local.sin_port = htons(kStatePort);
  check(::bind(client, reinterpret_cast<struct sockaddr*>(&local),
               sizeof(local)) == 0,
        "bridge udp client bind state port");

  struct sockaddr_in control {};
  control.sin_family = AF_INET;
  control.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  control.sin_port = htons(kControlPort);

  auto handler = [&](const std::string& text) -> std::string {
    bool set_origin = false;
    bool ping = false;
    std::string decode_error;
    if (!go2_nav::decodeNativeCommand(text, &set_origin, &ping,
                                      &decode_error)) {
      return std::string();
    }
    if (set_origin) {
      go2_nav::VinsReason result = go2_nav::VinsReason::NoVins;
      if (core.setOrigin(monotonicNow(), &result)) {
        return go2_nav::encodeOriginOk(core.extrinsic().measured);
      }
      return go2_nav::encodeOriginFail(result);
    }
    if (ping) {
      return go2_nav::encodeState(core.makeState(monotonicNow()));
    }
    return std::string();
  };

  const std::string origin_command = go2_nav::encodeSetOriginCommand();
  check(::sendto(client, origin_command.data(), origin_command.size(), 0,
                 reinterpret_cast<struct sockaddr*>(&control),
                 sizeof(control)) ==
            static_cast<ssize_t>(origin_command.size()),
        "bridge udp send SET_ORIGIN");
  check(server.pollOnce(100, handler, &error),
        "bridge udp server poll origin: " + error);
  std::string response;
  check(recvWithTimeout(client, &response) == 0, "bridge udp ORIGIN response");
  check(response.find("ORIGIN_OK 0") != std::string::npos,
        "bridge udp ORIGIN_OK measured false");

  const std::string ping = go2_nav::encodePingCommand();
  check(::sendto(client, ping.data(), ping.size(), 0,
                 reinterpret_cast<struct sockaddr*>(&control),
                 sizeof(control)) == static_cast<ssize_t>(ping.size()),
        "bridge udp send PING");
  check(server.pollOnce(100, handler, &error),
        "bridge udp server poll ping: " + error);
  check(recvWithTimeout(client, &response) == 0, "bridge udp STATE response");
  go2_nav::VinsBridgeMessage message;
  std::string decode_error;
  check(go2_nav::decodeBridgeMessage(response, &message, &decode_error),
        "bridge udp STATE decode: " + decode_error);
  check(message.type == go2_nav::VinsBridgeMessageType::State &&
            message.state.valid,
        "bridge udp STATE valid");
  check(!message.state.measured, "bridge udp STATE measured false");
  check(std::abs(message.state.vx) < 1e-9 &&
            std::abs(message.state.vy) < 1e-9 &&
            std::abs(message.state.wz) < 1e-9,
        "bridge udp STATE velocity fields");

  ::close(client);
  server.close();
}

}  // namespace

int main() {
  testExtrinsicConfig();
  testCoreExtrinsicRotation();
  testCoreNoNav();
  testCoreOriginAndState();
  testCoreStaleJumpReset();
  testHealthHeaderStallAndRecovery();
  testHealthLowFeaturesAndSlowDrift();
  testCoreNonFinite();
  testUdpLoopback();

  if (failures != 0) {
    std::cerr << "go2_vins_bridge selftest failures: " << failures << "\n";
    return 1;
  }
  std::cout << "all go2_vins_bridge selftests passed; no ROS/DDS initialized\n";
  return 0;
}
