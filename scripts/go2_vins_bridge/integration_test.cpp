#include "bridge_core.hpp"
#include "udp_bridge_server.hpp"

#include "native_motion_core.hpp"
#include "vins_udp_client.hpp"
#include "vins_wire.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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

class FakeBackend final : public go2_native_motion::Backend {
 public:
  go2_native_motion::StateSnapshot snapshot() override {
    go2_native_motion::StateSnapshot state;
    state.received = true;
    state.finite = true;
    state.seq = ++seq_;
    state.age_s = 0.01;
    state.position = {0.0, 0.0, 0.30};
    state.yaw_rad = 0.0;
    return state;
  }

  int32_t move(float /*vx*/, float vy, float /*vyaw*/) override {
    ++move_calls;
    if (std::abs(vy) > 1e-12) {
      ++lateral_calls;
    }
    return 0;
  }

  int32_t stopMove() override {
    ++stop_calls;
    return 0;
  }

  std::string lastError() override { return std::string(); }

  int move_calls = 0;
  int lateral_calls = 0;
  int stop_calls = 0;

 private:
  uint64_t seq_ = 0;
};

bool noCmdAfterStop(const std::string& csv) {
  const std::size_t stop = csv.find("stop_request");
  if (stop == std::string::npos) {
    return false;
  }
  return csv.find(",cmd,", stop) == std::string::npos;
}

}  // namespace

int main() {
  constexpr uint16_t kControlPort = 47401;
  constexpr uint16_t kStatePort = 47402;
  constexpr double kRunS = 1.6;

  go2_vins_bridge::BridgeConfig bridge_config;
  bridge_config.vins_timeout_s = 0.5;
  go2_vins_bridge::BridgeCore bridge_core(bridge_config);

  go2_vins_bridge::UdpBridgeServer server(kControlPort, kStatePort);
  std::string error;
  check(server.open(&error), "integration bridge server open: " + error);

  go2_native_motion::VinsUdpClient client(kControlPort, kStatePort);
  check(client.open(&error), "integration native client open: " + error);

  std::atomic<bool> stop_bridge{false};
  std::atomic<bool> feed_vins{true};

  std::thread feeder([&]() {
    double header = 1.0;
    while (feed_vins.load()) {
      const double receive = monotonicNow();
      go2_vins_bridge::VinsSample sample;
      sample.header_stamp_s = header;
      sample.receive_stamp_s = receive;
      sample.camera.stamp_s = receive;
      sample.camera.world_camera.position = go2_nav::Vec3{0.0, 0.0, 0.0};
      sample.camera.world_camera.rotation = go2_nav::Mat3::identity();
      sample.camera.valid_input = true;
      bridge_core.onVinsSample(sample, receive);
      header += 0.02;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

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
      if (bridge_core.setOrigin(monotonicNow(), &reason)) {
        return go2_nav::encodeOriginOk();
      }
      return go2_nav::encodeOriginFail(reason);
    }
    if (ping) {
      return go2_nav::encodeState(bridge_core.makeState(monotonicNow()));
    }
    return std::string();
  };

  std::thread command_thread([&]() {
    while (!stop_bridge.load()) {
      std::string poll_error;
      if (!server.pollOnce(50, handler, &poll_error)) {
        break;
      }
    }
  });

  std::thread state_thread([&]() {
    while (!stop_bridge.load()) {
      server.sendState(
          go2_nav::encodeState(bridge_core.makeState(monotonicNow())),
          nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  FakeBackend backend;
  std::ostringstream log;
  std::ostringstream csv;
  go2_native_motion::writeCsvHeader(csv);
  go2_native_motion::Config config;
  config.control_source = go2_native_motion::ControlSource::Vins;
  config.vins_timeout_s = 0.5;
  config.csv_path = "integration.csv";
  go2_native_motion::SessionState session_state;

  const double start_s = monotonicNow();
  struct ScriptEntry {
    double at_s;
    std::string line;
  };
  const std::vector<ScriptEntry> script = {
      {0.0, "origin"}, {0.1, "plan 0.5 0"}, {0.6, "status"}, {kRunS - 0.2, "quit"}};
  std::size_t script_index = 0;

  go2_native_motion::Runtime runtime;
  runtime.now_s = [] { return monotonicNow(); };
  runtime.sleep_s = [](double seconds) {
    if (seconds > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    }
  };
  runtime.interrupted = [] { return false; };
  runtime.poll_command = [&](go2_native_motion::Command& command) {
    if (script_index >= script.size()) {
      return false;
    }
    if (monotonicNow() - start_s < script[script_index].at_s) {
      return false;
    }
    command = go2_native_motion::parseCommand(script[script_index].line);
    ++script_index;
    return true;
  };

  std::thread stopper([&]() {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(kRunS * 0.35));
    feed_vins.store(false);
  });

  const int exit_code = go2_native_motion::runInteractiveSession(
      config, backend, &client, runtime, session_state, log, csv);

  stopper.join();
  stop_bridge.store(true);
  command_thread.join();
  state_thread.join();
  feeder.join();
  client.close();
  server.close();

  const std::string csv_text = csv.str();
  check(exit_code == 0, "integration session exit 0");
  check(backend.move_calls > 0, "integration plan sent Moves");
  check(backend.lateral_calls == 0, "integration no lateral Move");
  check(backend.stop_calls >= 1, "integration stop called");
  check(csv_text.find("control_source") != std::string::npos,
        "integration CSV header");
  check(csv_text.find("VINS_INVALID") != std::string::npos,
        "integration stale VINS stops");
  check(csv_text.find("vins_origin_failed") == std::string::npos,
        "integration origin succeeded");
  check(noCmdAfterStop(csv_text), "integration no cmd after stop");

  if (failures != 0) {
    std::cerr << "go2_vins_bridge integration failures: " << failures << "\n";
    return 1;
  }
  std::cout << "all go2_vins_bridge integration tests passed; "
               "no ROS/DDS initialized\n";
  return 0;
}
