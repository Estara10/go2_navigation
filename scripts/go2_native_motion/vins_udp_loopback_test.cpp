#include "vins_udp_client.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "vins_wire.hpp"

namespace {

constexpr uint16_t kControlPort = 47101;
constexpr uint16_t kStatePort = 47102;

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

void sendToState(int socket_fd, const std::string& text) {
  struct sockaddr_in destination {};
  destination.sin_family = AF_INET;
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  destination.sin_port = htons(kStatePort);
  ::sendto(socket_fd, text.data(), text.size(), 0,
           reinterpret_cast<struct sockaddr*>(&destination),
           sizeof(destination));
}

void runFakeBridge(std::atomic<bool>* stop, std::atomic<bool>* ready) {
  const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    ready->store(true);
    return;
  }
  struct sockaddr_in local {};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local.sin_port = htons(kControlPort);
  if (::bind(socket_fd, reinterpret_cast<struct sockaddr*>(&local),
             sizeof(local)) != 0) {
    ready->store(true);
    ::close(socket_fd);
    return;
  }
  ready->store(true);

  int origin_requests = 0;
  int ping_requests = 0;
  while (!stop->load()) {
    struct pollfd descriptor {};
    descriptor.fd = socket_fd;
    descriptor.events = POLLIN;
    const int ready = ::poll(&descriptor, 1, 100);
    if (ready <= 0) {
      continue;
    }
    char buffer[256];
    const ssize_t count = ::recvfrom(socket_fd, buffer, sizeof(buffer), 0,
                                     nullptr, nullptr);
    if (count <= 0) {
      continue;
    }
    bool set_origin = false;
    bool ping = false;
    std::string error;
    if (!go2_nav::decodeNativeCommand(
            std::string(buffer, static_cast<std::size_t>(count)), &set_origin,
            &ping, &error)) {
      continue;
    }
    if (set_origin) {
      ++origin_requests;
      if (origin_requests == 1) {
        sendToState(socket_fd, go2_nav::encodeOriginOk());
        go2_nav::VinsWireState state;
        state.received = true;
        state.seq = 1;
        state.stamp_s = monotonicNow();
        state.x = 1.25;
        state.y = -0.5;
        state.z = 0.0;
        state.yaw = 0.3;
        state.valid = true;
        state.reason = go2_nav::VinsReason::Ok;
        sendToState(socket_fd, go2_nav::encodeState(state));
      } else {
        sendToState(socket_fd,
                    go2_nav::encodeOriginFail(go2_nav::VinsReason::NoVins));
      }
    } else if (ping) {
      ++ping_requests;
      go2_nav::VinsWireState state;
      state.received = true;
      state.seq = static_cast<uint64_t>(10 + ping_requests);
      state.stamp_s = monotonicNow();
      state.x = 2.0;
      state.y = 0.25;
      state.yaw = -0.1;
      state.valid = true;
      state.reason = go2_nav::VinsReason::Ok;
      sendToState(socket_fd, go2_nav::encodeState(state));
    }
  }
  ::close(socket_fd);
}

bool waitForState(go2_native_motion::VinsUdpClient* client,
                  go2_nav::VinsWireState* state, uint64_t minimum_seq) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    *state = client->snapshot();
    if (state->received && state->seq >= minimum_seq) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

int main() {
  std::atomic<bool> stop{false};
  std::atomic<bool> ready{false};
  std::thread bridge(runFakeBridge, &stop, &ready);
  const auto ready_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!ready.load() &&
         std::chrono::steady_clock::now() < ready_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  go2_native_motion::VinsUdpClient client(kControlPort, kStatePort);
  std::string error;
  check(client.open(&error), "udp client open: " + error);

  check(client.requestOrigin(&error, 1.0),
        "udp ORIGIN_OK handshake: " + error);

  go2_nav::VinsWireState state;
  check(waitForState(&client, &state, 1), "udp STATE received");
  check(state.valid, "udp STATE valid");
  check(state.reason == go2_nav::VinsReason::Ok, "udp STATE reason OK");
  check(std::abs(state.x - 1.25) < 1e-9, "udp STATE x");
  check(std::abs(state.y + 0.5) < 1e-9, "udp STATE y");
  check(std::abs(state.yaw - 0.3) < 1e-9, "udp STATE yaw");

  error.clear();
  check(!client.requestOrigin(&error, 1.0), "udp ORIGIN_FAIL rejected");
  check(error == "NO_VINS", "udp ORIGIN_FAIL reason: " + error);

  client.requestPing();
  check(waitForState(&client, &state, 11), "udp PING response STATE");
  check(std::abs(state.x - 2.0) < 1e-9, "udp PING STATE x");

  client.close();
  stop.store(true);
  bridge.join();

  if (failures != 0) {
    std::cerr << "vins_udp_loopback failures: " << failures << "\n";
    return 1;
  }
  std::cout << "all vins_udp_loopback tests passed; no DDS initialized\n";
  return 0;
}
