#ifndef GO2_NAV_CORE_VINS_WIRE_HPP
#define GO2_NAV_CORE_VINS_WIRE_HPP

#include <cstdint>
#include <string>

namespace go2_nav {

// Reasons reported by the bridge. OK is the only valid-state reason.
enum class VinsReason {
  Ok,
  NoNav,
  NoVins,
  Stale,
  NonFinite,
  Jump,
  Reset,
  Unknown,
};

const char* vinsReasonName(VinsReason reason);
bool parseVinsReason(const std::string& text, VinsReason* reason);

// One bridge-to-native STATE payload. stamp_s is the bridge's PC
// CLOCK_MONOTONIC seconds. x/y/z/yaw are base_link in the bridge's nav frame.
struct VinsWireState {
  bool received = false;
  uint64_t seq = 0;
  double stamp_s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  bool valid = false;
  VinsReason reason = VinsReason::NoVins;
};

enum class VinsBridgeMessageType {
  State,
  OriginOk,
  OriginFail,
  Invalid,
};

struct VinsBridgeMessage {
  VinsBridgeMessageType type = VinsBridgeMessageType::Invalid;
  VinsWireState state;
  VinsReason reason = VinsReason::Unknown;
};

// Native -> bridge commands. One command per datagram; trailing whitespace is
// accepted.
std::string encodeSetOriginCommand();
std::string encodePingCommand();
bool decodeNativeCommand(const std::string& text, bool* set_origin, bool* ping,
                         std::string* error);

// Bridge -> native messages:
//   ORIGIN_OK
//   ORIGIN_FAIL <reason>
//   STATE <seq> <stamp> <x> <y> <z> <yaw> <valid> <reason>
std::string encodeOriginOk();
std::string encodeOriginFail(VinsReason reason);
std::string encodeState(const VinsWireState& state);
bool decodeBridgeMessage(const std::string& text, VinsBridgeMessage* message,
                         std::string* error);

}  // namespace go2_nav

#endif
