#ifndef GO2_NATIVE_MOTION_VINS_UDP_CLIENT_HPP
#define GO2_NATIVE_MOTION_VINS_UDP_CLIENT_HPP

#include <chrono>
#include <cstdint>
#include <string>

#include <netinet/in.h>

#include "native_motion_core.hpp"

namespace go2_native_motion {

// Minimal UDP client for the VINS bridge. It binds 127.0.0.1:<state_port> for
// STATE/ORIGIN responses and sends SET_ORIGIN/PING to 127.0.0.1:<control_port>.
// It never initializes ROS or DDS.
class VinsUdpClient final : public VinsClient {
 public:
  VinsUdpClient(uint16_t control_port, uint16_t state_port);
  ~VinsUdpClient() override;

  bool open(std::string* error);
  void close();

  go2_nav::VinsWireState snapshot() override;
  bool requestOrigin(std::string* error, double timeout_s) override;
  void requestPing() override;

 private:
  bool sendCommand(const std::string& command, std::string* error);
  void drain();

  uint16_t control_port_;
  uint16_t state_port_;
  int socket_ = -1;
  struct sockaddr_in control_address_ {};
  go2_nav::VinsWireState state_;
  bool origin_ok_ = false;
  bool origin_failed_ = false;
  std::string origin_error_;
};

}  // namespace go2_native_motion

#endif
