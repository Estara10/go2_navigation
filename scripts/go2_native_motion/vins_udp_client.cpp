#include "vins_udp_client.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>

#include "vins_wire.hpp"

namespace go2_native_motion {

VinsUdpClient::VinsUdpClient(uint16_t control_port, uint16_t state_port)
    : control_port_(control_port), state_port_(state_port) {}

VinsUdpClient::~VinsUdpClient() { close(); }

bool VinsUdpClient::open(std::string* error) {
  close();
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0) {
    if (error != nullptr) {
      *error = std::string("socket: ") + std::strerror(errno);
    }
    return false;
  }

  struct sockaddr_in local {};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local.sin_port = htons(state_port_);
  if (::bind(socket_, reinterpret_cast<struct sockaddr*>(&local),
             sizeof(local)) != 0) {
    if (error != nullptr) {
      *error = "bind 127.0.0.1:" + std::to_string(state_port_) + ": " +
               std::strerror(errno);
    }
    close();
    return false;
  }

  const int flags = ::fcntl(socket_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) != 0) {
    if (error != nullptr) {
      *error = std::string("fcntl O_NONBLOCK: ") + std::strerror(errno);
    }
    close();
    return false;
  }

  control_address_ = sockaddr_in{};
  control_address_.sin_family = AF_INET;
  control_address_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  control_address_.sin_port = htons(control_port_);
  return true;
}

void VinsUdpClient::close() {
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool VinsUdpClient::sendCommand(const std::string& command,
                                std::string* error) {
  if (socket_ < 0) {
    if (error != nullptr) {
      *error = "VINS UDP socket is not open";
    }
    return false;
  }
  const ssize_t sent = ::sendto(
      socket_, command.data(), command.size(), 0,
      reinterpret_cast<const struct sockaddr*>(&control_address_),
      sizeof(control_address_));
  if (sent != static_cast<ssize_t>(command.size())) {
    if (error != nullptr) {
      *error = std::string("sendto: ") + std::strerror(errno);
    }
    return false;
  }
  return true;
}

void VinsUdpClient::drain() {
  if (socket_ < 0) {
    return;
  }
  for (;;) {
    char buffer[512];
    const ssize_t count = ::recvfrom(socket_, buffer, sizeof(buffer), 0,
                                     nullptr, nullptr);
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return;
      }
      return;
    }
    go2_nav::VinsBridgeMessage message;
    std::string error;
    if (!go2_nav::decodeBridgeMessage(
            std::string(buffer, static_cast<std::size_t>(count)), &message,
            &error)) {
      continue;
    }
    switch (message.type) {
      case go2_nav::VinsBridgeMessageType::State:
        state_ = message.state;
        break;
      case go2_nav::VinsBridgeMessageType::OriginOk:
        origin_ok_ = true;
        break;
      case go2_nav::VinsBridgeMessageType::OriginFail:
        origin_failed_ = true;
        origin_error_ = go2_nav::vinsReasonName(message.reason);
        break;
      case go2_nav::VinsBridgeMessageType::Invalid:
        break;
    }
  }
}

go2_nav::VinsWireState VinsUdpClient::snapshot() {
  drain();
  return state_;
}

bool VinsUdpClient::requestOrigin(std::string* error, double timeout_s) {
  if (socket_ < 0) {
    if (error != nullptr) {
      *error = "VINS UDP socket is not open";
    }
    return false;
  }
  if (!(timeout_s > 0.0)) {
    if (error != nullptr) {
      *error = "invalid ORIGIN timeout";
    }
    return false;
  }
  origin_ok_ = false;
  origin_failed_ = false;
  origin_error_.clear();
  if (!sendCommand(go2_nav::encodeSetOriginCommand(), error)) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<
                            std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(timeout_s));
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    const int timeout_ms =
        std::max(1, std::min(1000, static_cast<int>(remaining.count())));
    struct pollfd descriptor {};
    descriptor.fd = socket_;
    descriptor.events = POLLIN;
    const int ready = ::poll(&descriptor, 1, timeout_ms);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (error != nullptr) {
        *error = std::string("poll: ") + std::strerror(errno);
      }
      return false;
    }
    if (ready == 0) {
      continue;
    }
    drain();
    if (origin_ok_) {
      return true;
    }
    if (origin_failed_) {
      if (error != nullptr) {
        *error = origin_error_;
      }
      return false;
    }
  }
  if (error != nullptr) {
    *error = "ORIGIN response timeout";
  }
  return false;
}

void VinsUdpClient::requestPing() {
  std::string ignored;
  sendCommand(go2_nav::encodePingCommand(), &ignored);
}

}  // namespace go2_native_motion
