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
  check(std::abs(state.x) < 1e-12 && std::abs(state.y) < 1e-12,
        "bridge origin x/y zero");

  const auto moved = makeSample(1.1, 0.1, 1.0, 0.0, 0.2);
  core.onVinsSample(moved, 0.1);
  state = core.makeState(0.1);
  check(state.valid, "bridge moved state valid");
  check(std::abs(state.x - 1.0) < 1e-9, "bridge moved x");
  check(std::abs(state.y) < 1e-9, "bridge moved y");
  check(std::abs(state.yaw - 0.2) < 1e-9, "bridge moved yaw");
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
  state = core.makeState(1.0);
  check(state.valid && std::abs(state.x - 0.1) < 1e-9,
        "bridge fresh sample after stale");

  core.onVinsSample(makeSample(1.2, 1.1, 2.0, 0.0, 0.0), 1.1);
  state = core.makeState(1.1);
  check(!state.valid && state.reason == go2_nav::VinsReason::Jump,
        "bridge position jump JUMP");

  core.onVinsSample(makeSample(1.0, 1.2, 2.0, 0.0, 0.0), 1.2);
  state = core.makeState(1.2);
  check(!state.valid && state.reason == go2_nav::VinsReason::Reset,
        "bridge header reset RESET");
  reason = go2_nav::VinsReason::Ok;
  check(!core.setOrigin(1.2, &reason), "bridge origin after reset fails");
  check(reason == go2_nav::VinsReason::Reset,
        "bridge origin after reset reason RESET");
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
        return go2_nav::encodeOriginOk();
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
  check(response.find("ORIGIN_OK") != std::string::npos,
        "bridge udp ORIGIN_OK");

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
  testCoreNonFinite();
  testUdpLoopback();

  if (failures != 0) {
    std::cerr << "go2_vins_bridge selftest failures: " << failures << "\n";
    return 1;
  }
  std::cout << "all go2_vins_bridge selftests passed; no ROS/DDS initialized\n";
  return 0;
}
