#include "vins_wire.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace go2_nav {
namespace {

std::string trim(const std::string& text) {
  const std::size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::string();
  }
  const std::size_t last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::vector<std::string> splitWhitespace(const std::string& text) {
  std::vector<std::string> tokens;
  std::istringstream stream(text);
  std::string token;
  while (stream >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

bool parseDoubleToken(const std::string& text, double* value) {
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

bool parseUnsignedToken(const std::string& text, uint64_t* value) {
  if (text.empty() || text.find('-') != std::string::npos) {
    return false;
  }
  try {
    std::size_t position = 0;
    const unsigned long long parsed = std::stoull(text, &position);
    if (position != text.size()) {
      return false;
    }
    *value = static_cast<uint64_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

const char* vinsReasonName(VinsReason reason) {
  switch (reason) {
    case VinsReason::Ok:
      return "OK";
    case VinsReason::NoNav:
      return "NO_NAV";
    case VinsReason::NoVins:
      return "NO_VINS";
    case VinsReason::Stale:
      return "STALE";
    case VinsReason::NonFinite:
      return "NONFINITE";
    case VinsReason::Jump:
      return "JUMP";
    case VinsReason::Reset:
      return "RESET";
    case VinsReason::Unknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

bool parseVinsReason(const std::string& text, VinsReason* reason) {
  if (reason == nullptr) {
    return false;
  }
  if (text == "OK") {
    *reason = VinsReason::Ok;
  } else if (text == "NO_NAV") {
    *reason = VinsReason::NoNav;
  } else if (text == "NO_VINS") {
    *reason = VinsReason::NoVins;
  } else if (text == "STALE") {
    *reason = VinsReason::Stale;
  } else if (text == "NONFINITE") {
    *reason = VinsReason::NonFinite;
  } else if (text == "JUMP") {
    *reason = VinsReason::Jump;
  } else if (text == "RESET") {
    *reason = VinsReason::Reset;
  } else {
    return false;
  }
  return true;
}

std::string encodeSetOriginCommand() { return "SET_ORIGIN\n"; }

std::string encodePingCommand() { return "PING\n"; }

bool decodeNativeCommand(const std::string& text, bool* set_origin, bool* ping,
                         std::string* error) {
  if (set_origin == nullptr || ping == nullptr) {
    if (error != nullptr) {
      *error = "null output";
    }
    return false;
  }
  *set_origin = false;
  *ping = false;
  const std::string command = trim(text);
  if (command == "SET_ORIGIN") {
    *set_origin = true;
    return true;
  }
  if (command == "PING") {
    *ping = true;
    return true;
  }
  if (error != nullptr) {
    *error = "unknown native command: " + command;
  }
  return false;
}

std::string encodeOriginOk() { return "ORIGIN_OK\n"; }

std::string encodeOriginFail(VinsReason reason) {
  return "ORIGIN_FAIL " + std::string(vinsReasonName(reason)) + "\n";
}

std::string encodeState(const VinsWireState& state) {
  std::ostringstream out;
  out << "STATE " << state.seq << ' ' << std::fixed << std::setprecision(9)
      << state.stamp_s << ' ' << state.x << ' ' << state.y << ' ' << state.z
      << ' ' << state.yaw << ' ' << (state.valid ? 1 : 0) << ' '
      << vinsReasonName(state.reason) << '\n';
  return out.str();
}

bool decodeBridgeMessage(const std::string& text, VinsBridgeMessage* message,
                         std::string* error) {
  if (message == nullptr) {
    if (error != nullptr) {
      *error = "null message output";
    }
    return false;
  }
  *message = VinsBridgeMessage{};
  const std::vector<std::string> tokens = splitWhitespace(trim(text));
  if (tokens.empty()) {
    if (error != nullptr) {
      *error = "empty bridge message";
    }
    return false;
  }

  if (tokens[0] == "ORIGIN_OK") {
    if (tokens.size() != 1) {
      if (error != nullptr) {
        *error = "ORIGIN_OK has extra fields";
      }
      return false;
    }
    message->type = VinsBridgeMessageType::OriginOk;
    return true;
  }

  if (tokens[0] == "ORIGIN_FAIL") {
    if (tokens.size() != 2) {
      if (error != nullptr) {
        *error = "ORIGIN_FAIL requires exactly one reason";
      }
      return false;
    }
    VinsReason reason = VinsReason::Unknown;
    if (!parseVinsReason(tokens[1], &reason) || reason == VinsReason::Ok) {
      if (error != nullptr) {
        *error = "invalid ORIGIN_FAIL reason: " + tokens[1];
      }
      return false;
    }
    message->type = VinsBridgeMessageType::OriginFail;
    message->reason = reason;
    return true;
  }

  if (tokens[0] != "STATE" || tokens.size() != 9) {
    if (error != nullptr) {
      *error = "expected STATE with 8 fields";
    }
    return false;
  }

  VinsWireState state;
  if (!parseUnsignedToken(tokens[1], &state.seq) ||
      !parseDoubleToken(tokens[2], &state.stamp_s) ||
      !parseDoubleToken(tokens[3], &state.x) ||
      !parseDoubleToken(tokens[4], &state.y) ||
      !parseDoubleToken(tokens[5], &state.z) ||
      !parseDoubleToken(tokens[6], &state.yaw)) {
    if (error != nullptr) {
      *error = "invalid numeric field in STATE";
    }
    return false;
  }
  if (tokens[7] != "0" && tokens[7] != "1") {
    if (error != nullptr) {
      *error = "STATE valid must be 0 or 1";
    }
    return false;
  }
  state.valid = tokens[7] == "1";
  VinsReason reason = VinsReason::Unknown;
  if (!parseVinsReason(tokens[8], &reason)) {
    if (error != nullptr) {
      *error = "invalid STATE reason: " + tokens[8];
    }
    return false;
  }
  if (state.valid && reason != VinsReason::Ok) {
    if (error != nullptr) {
      *error = "valid STATE must use reason OK";
    }
    return false;
  }
  if (!state.valid && reason == VinsReason::Ok) {
    if (error != nullptr) {
      *error = "invalid STATE cannot use reason OK";
    }
    return false;
  }
  state.reason = reason;
  state.received = true;
  message->type = VinsBridgeMessageType::State;
  message->state = state;
  return true;
}

}  // namespace go2_nav
