#ifndef GO2_VINS_BRIDGE_UDP_BRIDGE_SERVER_HPP
#define GO2_VINS_BRIDGE_UDP_BRIDGE_SERVER_HPP

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include <netinet/in.h>

namespace go2_vins_bridge {

// Receives SET_ORIGIN/PING on 127.0.0.1:<control_port> and sends responses and
// STATE datagrams to 127.0.0.1:<state_port>.
class UdpBridgeServer {
 public:
  UdpBridgeServer(uint16_t control_port, uint16_t state_port);
  ~UdpBridgeServer();

  bool open(std::string* error);
  void close();

  // Polls one command, calls handler(message), and sends the non-empty
  // response to the state port. Returns false only on socket failure.
  bool pollOnce(int timeout_ms,
                const std::function<std::string(const std::string&)>& handler,
                std::string* error);

  bool sendState(const std::string& message, std::string* error);
  bool isOpen() const { return socket_ >= 0; }

 private:
  uint16_t control_port_;
  uint16_t state_port_;
  int socket_ = -1;
  struct sockaddr_in state_address_ {};
  std::mutex send_mutex_;
};

}  // namespace go2_vins_bridge

#endif
