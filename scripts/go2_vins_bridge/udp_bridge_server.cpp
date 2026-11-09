#include "udp_bridge_server.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace go2_vins_bridge {

UdpBridgeServer::UdpBridgeServer(uint16_t control_port, uint16_t state_port)
    : control_port_(control_port), state_port_(state_port) {}

UdpBridgeServer::~UdpBridgeServer() { close(); }

bool UdpBridgeServer::open(std::string* error) {
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
  local.sin_port = htons(control_port_);
  if (::bind(socket_, reinterpret_cast<struct sockaddr*>(&local),
             sizeof(local)) != 0) {
    if (error != nullptr) {
      *error = "bind 127.0.0.1:" + std::to_string(control_port_) + ": " +
               std::strerror(errno);
    }
    close();
    return false;
  }

  state_address_ = sockaddr_in{};
  state_address_.sin_family = AF_INET;
  state_address_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  state_address_.sin_port = htons(state_port_);
  return true;
}

void UdpBridgeServer::close() {
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool UdpBridgeServer::sendState(const std::string& message,
                                std::string* error) {
  if (socket_ < 0) {
    if (error != nullptr) {
      *error = "bridge UDP socket is not open";
    }
    return false;
  }
  std::lock_guard<std::mutex> lock(send_mutex_);
  const ssize_t sent = ::sendto(
      socket_, message.data(), message.size(), 0,
      reinterpret_cast<const struct sockaddr*>(&state_address_),
      sizeof(state_address_));
  if (sent != static_cast<ssize_t>(message.size())) {
    if (error != nullptr) {
      *error = std::string("sendto state: ") + std::strerror(errno);
    }
    return false;
  }
  return true;
}

bool UdpBridgeServer::pollOnce(
    int timeout_ms,
    const std::function<std::string(const std::string&)>& handler,
    std::string* error) {
  if (socket_ < 0) {
    if (error != nullptr) {
      *error = "bridge UDP socket is not open";
    }
    return false;
  }
  struct pollfd descriptor {};
  descriptor.fd = socket_;
  descriptor.events = POLLIN;
  const int ready = ::poll(&descriptor, 1, timeout_ms);
  if (ready < 0) {
    if (errno == EINTR) {
      return true;
    }
    if (error != nullptr) {
      *error = std::string("poll: ") + std::strerror(errno);
    }
    return false;
  }
  if (ready == 0) {
    return true;
  }

  char buffer[512];
  const ssize_t count = ::recvfrom(socket_, buffer, sizeof(buffer), 0, nullptr,
                                   nullptr);
  if (count <= 0) {
    if (error != nullptr) {
      *error = std::string("recvfrom: ") + std::strerror(errno);
    }
    return false;
  }
  const std::string response =
      handler(std::string(buffer, static_cast<std::size_t>(count)));
  if (!response.empty()) {
    std::string send_error;
    if (!sendState(response, &send_error)) {
      if (error != nullptr) {
        *error = send_error;
      }
      return false;
    }
  }
  return true;
}

}  // namespace go2_vins_bridge
