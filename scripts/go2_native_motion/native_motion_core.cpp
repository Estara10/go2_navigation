#include "native_motion_core.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace go2_native_motion {

void MotionConsistencyMonitor::reset(
    double now_s, const std::array<double, 2>& vins_position) {
  initialized_ = std::isfinite(now_s) && std::isfinite(vins_position[0]) &&
                 std::isfinite(vins_position[1]);
  last_time_s_ = now_s;
  last_position_ = vins_position;
  command_distance_m_ = 0.0;
  vins_distance_m_ = 0.0;
  divergence_start_s_ = 0.0;
  divergence_active_ = false;
}

MotionConsistencyFault MotionConsistencyMonitor::observe(
    double now_s, const std::array<double, 2>& vins_position,
    double previous_command_speed_mps, double native_speed_mps,
    double vins_speed_mps) {
  if (!initialized_ || !std::isfinite(now_s) ||
      !std::isfinite(vins_position[0]) || !std::isfinite(vins_position[1]) ||
      !std::isfinite(previous_command_speed_mps) ||
      !std::isfinite(native_speed_mps) || !std::isfinite(vins_speed_mps) ||
      now_s <= last_time_s_) {
    return MotionConsistencyFault::None;
  }
  const double dt = now_s - last_time_s_;
  const double dx = vins_position[0] - last_position_[0];
  const double dy = vins_position[1] - last_position_[1];
  command_distance_m_ += std::max(0.0, previous_command_speed_mps) * dt;
  vins_distance_m_ += std::hypot(dx, dy);
  last_time_s_ = now_s;
  last_position_ = vins_position;

  // Ignore startup/settling until at least half a metre was commanded.
  if (command_distance_m_ >= 0.5 &&
      vins_distance_m_ > 1.15 * command_distance_m_) {
    return MotionConsistencyFault::VinsDisplacementExceedsCommandBound;
  }

  // SportModeState velocity is only a cross-check. Require a sustained
  // magnitude disagreement while moving; frame orientation is irrelevant.
  constexpr double kSpeedDisagreementMps = 0.45;
  constexpr double kSpeedDisagreementDurationS = 0.5;
  const bool disagree = std::max(native_speed_mps, vins_speed_mps) > 0.20 &&
      std::abs(native_speed_mps - vins_speed_mps) > kSpeedDisagreementMps;
  if (disagree) {
    if (!divergence_active_) {
      divergence_active_ = true;
      divergence_start_s_ = now_s;
    } else if (now_s - divergence_start_s_ >=
               kSpeedDisagreementDurationS) {
      return MotionConsistencyFault::VinsNativeSpeedDivergence;
    }
  } else {
    divergence_active_ = false;
  }
  return MotionConsistencyFault::None;
}

namespace {

constexpr double kScheduleEpsilonS = 1e-9;

std::string wallTimeUtc() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const std::time_t seconds = system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  char text[32];
  std::strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%S", &tm);
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  std::ostringstream out;
  out << text << '.' << std::setw(3) << std::setfill('0') << ms.count() << 'Z';
  return out.str();
}

std::string number(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

std::string csvField(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped += c;
    }
  }
  escaped += '"';
  return escaped;
}

std::string trim(const std::string& text) {
  const std::size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string toLower(const std::string& text) {
  std::string lowered = text;
  for (char& c : lowered) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return lowered;
}

std::vector<std::string> splitWhitespace(const std::string& text) {
  std::vector<std::string> tokens;
  std::size_t position = 0;
  while (position < text.size()) {
    while (position < text.size() &&
           std::isspace(static_cast<unsigned char>(text[position])) != 0) {
      ++position;
    }
    const std::size_t start = position;
    while (position < text.size() &&
           std::isspace(static_cast<unsigned char>(text[position])) == 0) {
      ++position;
    }
    if (position > start) {
      tokens.push_back(text.substr(start, position - start));
    }
  }
  return tokens;
}

bool parseFiniteDouble(const std::string& text, double& value) {
  try {
    std::size_t position = 0;
    const double parsed = std::stod(text, &position);
    if (position != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

struct RowContext {
  const StateSnapshot* state = nullptr;
  const Origin* origin = nullptr;
  const Task* task = nullptr;
  bool has_cmd = false;
  double vx = 0.0;
  double vy = 0.0;
  double vyaw = 0.0;
  bool has_api = false;
  int32_t api = 0;
  const char* event = nullptr;
  std::string detail;
  bool vins_control = false;
  const go2_nav::VinsWireState* vins = nullptr;
  bool vins_valid = false;
  std::string vins_reason;
  bool vins_measured = false;
  bool measured_override = false;
  bool has_local = false;
  LocalPose local;
  bool has_vins_age = false;
  double vins_age_s = 0.0;
};

void writeRow(std::ostream& csv, double mono_s, const char* type,
              const RowContext& context) {
  std::vector<std::string> fields;
  fields.reserve(31);
  fields.push_back(wallTimeUtc());
  fields.push_back(number(mono_s, 6));
  fields.push_back(type == nullptr ? "" : type);
  fields.push_back(taskName(context.task == nullptr ? TaskType::None
                                                   : context.task->type));

  if (context.state != nullptr && context.state->received) {
    fields.push_back(std::to_string(context.state->seq));
    fields.push_back(number(context.state->age_s, 6));
  } else {
    fields.push_back("");
    fields.push_back("");
  }

  if (!context.vins_control && context.origin != nullptr &&
      context.origin->set) {
    fields.push_back(number(context.origin->native_x, 6));
    fields.push_back(number(context.origin->native_y, 6));
    fields.push_back(number(context.origin->native_yaw, 6));
  } else {
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
  }

  if (context.task != nullptr &&
      (context.task->type == TaskType::Goto ||
       context.task->type == TaskType::Plan)) {
    fields.push_back(number(context.task->target_local_x, 6));
    fields.push_back(number(context.task->target_local_y, 6));
    fields.push_back(number(context.task->target_local_yaw_deg, 6));
    if (context.vins_control) {
      fields.push_back("");
      fields.push_back("");
      fields.push_back("");
    } else {
      fields.push_back(number(context.task->target_native_x, 6));
      fields.push_back(number(context.task->target_native_y, 6));
      fields.push_back(number(context.task->target_native_yaw, 6));
    }
  } else if (context.task != nullptr &&
             context.task->type == TaskType::Yaw) {
    fields.push_back("");
    fields.push_back("");
    fields.push_back(number(context.task->target_local_yaw_deg, 6));
    fields.push_back("");
    fields.push_back("");
    if (context.vins_control) {
      fields.push_back("");
    } else {
      fields.push_back(number(context.task->target_native_yaw, 6));
    }
  } else {
    for (int i = 0; i < 6; ++i) {
      fields.push_back("");
    }
  }

  if (context.task != nullptr && context.task->type == TaskType::Pulse) {
    fields.push_back(number(context.task->pulse_vx_mps, 6));
    fields.push_back(number(context.task->pulse_duration_s, 6));
  } else {
    fields.push_back("");
    fields.push_back("");
  }

  if (context.has_local) {
    fields.push_back(number(context.local.x, 6));
    fields.push_back(number(context.local.y, 6));
    fields.push_back(number(context.local.heading * 180.0 / kPi, 6));
  } else if (!context.vins_control && context.origin != nullptr &&
             context.origin->set && context.state != nullptr &&
             context.state->received) {
    const LocalPose local = toLocalPose(*context.state, *context.origin);
    fields.push_back(number(local.x, 6));
    fields.push_back(number(local.y, 6));
    fields.push_back(number(local.heading * 180.0 / kPi, 6));
  } else {
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
  }

  if (context.has_cmd) {
    fields.push_back(number(context.vx, 6));
    fields.push_back(number(context.vy, 6));
    fields.push_back(number(context.vyaw, 6));
  } else {
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
  }

  if (context.state != nullptr && context.state->received) {
    for (const double value : context.state->position) {
      fields.push_back(number(value, 6));
    }
    for (const double value : context.state->velocity) {
      fields.push_back(number(value, 6));
    }
    fields.push_back(number(context.state->yaw_rad, 6));
  } else {
    for (int i = 0; i < 7; ++i) {
      fields.push_back("");
    }
  }

  fields.push_back(context.vins_control ? "vins" : "native");
  if (context.vins_control && context.vins != nullptr) {
    fields.push_back(context.vins_valid ? "1" : "0");
    fields.push_back(context.vins_reason);
    fields.push_back(std::to_string(context.vins->seq));
    fields.push_back(number(context.vins->stamp_s, 6));
    fields.push_back(context.has_vins_age
                         ? number(context.vins_age_s, 6)
                         : std::string());
    fields.push_back(number(context.vins->vx, 6));
    fields.push_back(number(context.vins->vy, 6));
    fields.push_back(number(context.vins->wz, 6));
    fields.push_back(context.vins_measured ? "1" : "0");
    fields.push_back(context.measured_override ? "1" : "0");
  } else {
    for (int i = 0; i < 10; ++i) {
      fields.push_back("");
    }
  }

  fields.push_back(context.has_api ? std::to_string(context.api) : "");
  fields.push_back(context.event == nullptr ? "" : context.event);
  fields.push_back(csvField(context.detail));

  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i != 0) {
      csv << ',';
    }
    csv << fields[i];
  }
  csv << '\n';
}

std::string staleDetail(const StateSnapshot& state) {
  std::ostringstream out;
  out << "SportModeState not fresh (received=" << state.received
      << ", finite=" << state.finite << ", age_s=" << number(state.age_s, 3)
      << ", limit_s=" << number(kMaxStateAgeS, 3) << ')';
  return out.str();
}

bool finiteVins(const go2_nav::VinsWireState& vins) {
  return std::isfinite(vins.stamp_s) && std::isfinite(vins.x) &&
         std::isfinite(vins.y) && std::isfinite(vins.z) &&
         std::isfinite(vins.yaw);
}

double vinsAgeS(const go2_nav::VinsWireState& vins, double now_s) {
  return vins.received ? now_s - vins.stamp_s
                       : std::numeric_limits<double>::infinity();
}

StateSnapshot snapshotFromVins(const go2_nav::VinsWireState& vins,
                               double now_s) {
  StateSnapshot snapshot;
  snapshot.received = vins.received;
  snapshot.finite = vins.valid;
  snapshot.seq = vins.seq;
  snapshot.age_s = vinsAgeS(vins, now_s);
  snapshot.position = {vins.x, vins.y, 0.0};
  snapshot.velocity = {vins.vx, vins.vy, 0.0};
  snapshot.yaw_rad = vins.yaw;
  return snapshot;
}

bool vinsUsable(const go2_nav::VinsWireState& vins, double now_s,
                double timeout_s) {
  if (!vins.received || !vins.valid || vins.reason != go2_nav::VinsReason::Ok ||
      !finiteVins(vins)) {
    return false;
  }
  const double age_s = vinsAgeS(vins, now_s);
  return std::isfinite(age_s) && age_s >= -0.05 && age_s <= timeout_s;
}

const char* vinsReasonForDisplay(const go2_nav::VinsWireState& vins,
                                 double now_s, double timeout_s) {
  if (vins.received && vins.valid && finiteVins(vins) &&
      vinsAgeS(vins, now_s) > timeout_s) {
    return go2_nav::vinsReasonName(go2_nav::VinsReason::Stale);
  }
  return go2_nav::vinsReasonName(vins.reason);
}

}  // namespace

bool validateConfig(const Config& config, std::string* error) {
  if (!std::isfinite(config.vx_mps) || config.vx_mps < kMinVxMps ||
      config.vx_mps > kMaxVxMps) {
    if (error != nullptr) {
      *error = "vx must be finite and within [" + number(kMinVxMps, 3) + ", " +
               number(kMaxVxMps, 3) + "] m/s";
    }
    return false;
  }
  if (config.vy_mps != 0.0 || config.vyaw_radps != 0.0) {
    if (error != nullptr) {
      *error = "straight command is forward-only: vy and vyaw must be 0";
    }
    return false;
  }
  if (!std::isfinite(config.duration_s) || config.duration_s < kMinDurationS ||
      config.duration_s > kMaxDurationS) {
    if (error != nullptr) {
      *error = "duration must be finite and within [" +
               number(kMinDurationS, 3) + ", " + number(kMaxDurationS, 3) + "] s";
    }
    return false;
  }
  if (!std::isfinite(config.goto_max_speed_mps) ||
      config.goto_max_speed_mps < kMinGotoSpeedMps ||
      config.goto_max_speed_mps > kMaxGotoSpeedMps) {
    if (error != nullptr) {
      *error = "goto max speed must be finite and within [" +
               number(kMinGotoSpeedMps, 3) + ", " +
               number(kMaxGotoSpeedMps, 3) + "] m/s";
    }
    return false;
  }
  if (!std::isfinite(config.goto_approach_speed_mps) ||
      config.goto_approach_speed_mps < kMinGotoSpeedMps ||
      config.goto_approach_speed_mps > config.goto_max_speed_mps) {
    if (error != nullptr) {
      *error = "goto approach speed must be finite and within [" +
               number(kMinGotoSpeedMps, 3) + ", " +
               number(config.goto_max_speed_mps, 3) + "] m/s";
    }
    return false;
  }
  if (config.csv_path.empty()) {
    if (error != nullptr) {
      *error = "csv path must not be empty";
    }
    return false;
  }
  return true;
}

bool isFresh(const StateSnapshot& state) {
  return state.received && state.finite && std::isfinite(state.age_s) &&
         state.age_s >= 0.0 && state.age_s <= kMaxStateAgeS;
}

Command parseCommand(const std::string& line) {
  Command command;
  command.raw = trim(line);
  if (command.raw.empty()) {
    return command;
  }
  const std::vector<std::string> tokens = splitWhitespace(command.raw);
  if (tokens.empty()) {
    return command;
  }
  const std::string name = toLower(tokens[0]);
  if (tokens.size() == 1 && name == "status") {
    command.type = CommandType::Status;
  } else if (tokens.size() == 1 && name == "origin") {
    command.type = CommandType::Origin;
  } else if (tokens.size() == 1 && name == "straight") {
    command.type = CommandType::Straight;
  } else if (tokens.size() == 1 && name == "stop") {
    command.type = CommandType::Stop;
  } else if (tokens.size() == 1 && name == "quit") {
    command.type = CommandType::Quit;
  } else if (tokens.size() == 1 && name == "help") {
    command.type = CommandType::Help;
  } else if (tokens.size() == 3 && name == "goto") {
    double x = 0.0;
    double y = 0.0;
    if (parseFiniteDouble(tokens[1], x) && parseFiniteDouble(tokens[2], y)) {
      command.type = CommandType::Goto;
      command.x = x;
      command.y = y;
    }
  } else if (tokens.size() == 3 && name == "plan") {
    double forward = 0.0;
    double left = 0.0;
    if (parseFiniteDouble(tokens[1], forward) &&
        parseFiniteDouble(tokens[2], left)) {
      command.type = CommandType::Plan;
      command.x = forward;
      command.y = left;
    }
  } else if (tokens.size() == 3 && name == "pulse") {
    double vx = 0.0;
    double duration = 0.0;
    if (parseFiniteDouble(tokens[1], vx) &&
        parseFiniteDouble(tokens[2], duration)) {
      command.type = CommandType::Pulse;
      command.pulse_vx_mps = vx;
      command.pulse_duration_s = duration;
    }
  } else if (tokens.size() == 2 && name == "yaw") {
    double angle = 0.0;
    if (parseFiniteDouble(tokens[1], angle)) {
      command.type = CommandType::Yaw;
      command.angle_deg = angle;
    }
  }
  return command;
}

const char* commandName(CommandType type) {
  switch (type) {
    case CommandType::Invalid:
      return "invalid";
    case CommandType::Help:
      return "help";
    case CommandType::Status:
      return "status";
    case CommandType::Origin:
      return "origin";
    case CommandType::Straight:
      return "straight";
    case CommandType::Pulse:
      return "pulse";
    case CommandType::Goto:
      return "goto";
    case CommandType::Plan:
      return "plan";
    case CommandType::Yaw:
      return "yaw";
    case CommandType::Stop:
      return "stop";
    case CommandType::Quit:
      return "quit";
  }
  return "unknown";
}

const char* taskName(TaskType type) {
  switch (type) {
    case TaskType::None:
      return "none";
    case TaskType::Straight:
      return "straight";
    case TaskType::Pulse:
      return "pulse";
    case TaskType::Goto:
      return "goto";
    case TaskType::Plan:
      return "plan";
    case TaskType::Yaw:
      return "yaw";
  }
  return "unknown";
}

const char* taskEndReasonName(TaskEndReason reason) {
  switch (reason) {
    case TaskEndReason::Arrived:
      return "ARRIVED";
    case TaskEndReason::Completed:
      return "COMPLETED";
    case TaskEndReason::Timeout:
      return "TIMEOUT";
    case TaskEndReason::Stopped:
      return "STOPPED";
    case TaskEndReason::Interrupted:
      return "INTERRUPTED";
    case TaskEndReason::StateStale:
      return "STATE_STALE";
    case TaskEndReason::VinsInvalid:
      return "VINS_INVALID";
    case TaskEndReason::MoveApiError:
      return "MOVE_API_ERROR";
    case TaskEndReason::Quit:
      return "QUIT";
  }
  return "UNKNOWN";
}

const char* controlSourceName(ControlSource source) {
  switch (source) {
    case ControlSource::Native:
      return "native";
    case ControlSource::Vins:
      return "vins";
  }
  return "unknown";
}

double wrapPi(double angle) {
  angle = std::fmod(angle, 2.0 * kPi);
  if (angle <= -kPi) {
    angle += 2.0 * kPi;
  } else if (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  return angle;
}

LocalPose toLocalPose(const StateSnapshot& state, const Origin& origin) {
  LocalPose local;
  const double dx = state.position[0] - origin.native_x;
  const double dy = state.position[1] - origin.native_y;
  const double c = std::cos(origin.native_yaw);
  const double s = std::sin(origin.native_yaw);
  local.x = c * dx + s * dy;
  local.y = -s * dx + c * dy;
  local.heading = wrapPi(state.yaw_rad - origin.native_yaw);
  return local;
}

void localToNative(const Origin& origin, double local_x, double local_y,
                   double* native_x, double* native_y) {
  const double c = std::cos(origin.native_yaw);
  const double s = std::sin(origin.native_yaw);
  if (native_x != nullptr) {
    *native_x = origin.native_x + c * local_x - s * local_y;
  }
  if (native_y != nullptr) {
    *native_y = origin.native_y + s * local_x + c * local_y;
  }
}

void writeCsvHeader(std::ostream& csv) {
  csv << "wall_time_utc,mono_s,record_type,task,state_seq,state_age_s,"
         "origin_native_x,origin_native_y,origin_native_yaw,"
         "target_local_x,target_local_y,target_local_yaw_deg,"
         "target_native_x,target_native_y,target_native_yaw,"
         "pulse_vx_mps,pulse_duration_s,"
         "local_pos_x,local_pos_y,local_heading_deg,"
         "cmd_vx_mps,cmd_vy_mps,cmd_vyaw_radps,"
         "native_pos_x,native_pos_y,native_pos_z,"
         "native_vel_x,native_vel_y,native_vel_z,native_yaw_raw,"
         "control_source,vins_valid,vins_reason,vins_seq,vins_stamp,"
         "vins_age_s,vins_vx,vins_vy,vins_wz,vins_measured,"
         "measured_override,"
         "api_return,event,detail\n";
}

int runInteractiveSession(const Config& config, Backend& backend,
                          VinsClient* vins_client, const Runtime& runtime,
                          SessionState& session_state, std::ostream& log,
                          std::ostream& csv) {
  const double session_t0 = runtime.now_s();
  StateSnapshot state;
  StateSnapshot control_state;
  go2_nav::VinsWireState vins_state;
  Origin origin;
  Task task;
  MotionConsistencyMonitor motion_consistency;
  double previous_command_speed_mps = 0.0;
  double next_control = 0.0;
  bool control_valid = false;
  bool control_local_available = false;
  bool control_measured = false;
  bool unmeasured_override = false;
  double control_yaw_rate_radps = 0.0;
  LocalPose control_local;
  std::string control_reason = "NO_VINS";
  session_state.stop_failed = false;

  auto refreshControl = [&](double now_s) {
    state = backend.snapshot();
    if (config.control_source == ControlSource::Native) {
      control_state = state;
      control_valid = isFresh(state);
      control_reason = control_valid ? "OK" : "STALE";
      control_local_available = origin.set && state.received;
      control_measured = false;
      control_yaw_rate_radps = 0.0;
      if (control_local_available) {
        control_local = toLocalPose(state, origin);
      }
      return;
    }

    vins_state = vins_client != nullptr ? vins_client->snapshot()
                                        : go2_nav::VinsWireState{};
    control_state = snapshotFromVins(vins_state, now_s);
    control_valid = vinsUsable(vins_state, now_s, config.vins_timeout_s);
    control_reason =
        control_valid
            ? go2_nav::vinsReasonName(go2_nav::VinsReason::Ok)
            : vinsReasonForDisplay(vins_state, now_s, config.vins_timeout_s);
    control_local_available = vins_state.received;
    control_local = LocalPose{vins_state.x, vins_state.y, vins_state.yaw};
    control_measured = vins_state.measured;
    control_yaw_rate_radps = vins_state.wz;
  };

  auto writeEvent = [&](const char* name, const std::string& detail,
                        const Task* task_override = nullptr,
                        bool has_api = false, int32_t api = 0) {
    RowContext context;
    context.state = &state;
    context.origin = &origin;
    context.task = task_override == nullptr ? &task : task_override;
    context.has_api = has_api;
    context.api = api;
    context.event = name;
    context.detail = detail;
    context.vins_control = config.control_source == ControlSource::Vins;
    context.vins = context.vins_control ? &vins_state : nullptr;
    context.vins_valid = control_valid;
    context.vins_reason = control_reason;
    context.vins_measured = control_measured;
    context.measured_override = unmeasured_override;
    context.has_local = control_local_available;
    context.local = control_local;
    context.has_vins_age =
        context.vins_control && vins_state.received;
    context.vins_age_s =
        context.has_vins_age ? runtime.now_s() - vins_state.stamp_s : 0.0;
    writeRow(csv, runtime.now_s() - session_t0, "event", context);
    csv.flush();
  };

  log << std::fixed << std::setprecision(3);
  log << "等待有效原生 SportModeState (期限 " << kMaxStateAgeS << " s)...\n";

  bool ready = false;
  while (!ready) {
    if (runtime.interrupted()) {
      writeEvent("end", "INTERRUPTED_BEFORE_STATE");
      log << "状态等待期间收到中断；未发送运动指令。\n";
      return 1;
    }
    state = backend.snapshot();
    if (isFresh(state)) {
      ready = true;
      break;
    }
    if (runtime.now_s() - session_t0 >= kStartupStateWaitS) {
      std::ostringstream detail;
      detail << "no fresh SportModeState within " << kStartupStateWaitS
             << " s (received=" << state.received
             << ", finite=" << state.finite
             << ", age_s=" << number(state.age_s, 3) << ')';
      writeEvent("end", "STARTUP_TIMEOUT: " + detail.str());
      log << "启动等待超时: " << detail.str() << "\n";
      log << "未发送运动指令。\n";
      return 1;
    }
    runtime.sleep_s(0.05);
  }

  log << "原生状态已就绪: seq=" << state.seq << " age=" << state.age_s
      << " s\n";
  log << "原生位置估计(position, 字段原样): " << state.position[0] << ", "
      << state.position[1] << ", " << state.position[2] << "\n";
  log << "原生线速度估计(velocity, 字段原样): " << state.velocity[0] << ", "
      << state.velocity[1] << ", " << state.velocity[2] << "\n";
  log << "原生航向(IMU rpy[2], 字段原样；单位待本机核实): " << state.yaw_rad
      << "\n";
  log << "位置/速度/航向均为估计反馈，坐标系和单位待本机核实，不是独立测量真值。\n";
  log << "控制来源: " << controlSourceName(config.control_source);
  if (config.control_source == ControlSource::Vins) {
    log << "（plan/goto/yaw 使用 VINS BodyState；包超时 "
        << config.vins_timeout_s << " s；原生 SportModeState 只作状态显示/停止）";
  } else {
    log << "（plan/goto/yaw 使用原生 SportModeState）";
  }
  log << "\n";
  writeEvent("state_ready", "");

  log << "本次 straight 参数: vx=" << config.vx_mps << " m/s, duration="
      << config.duration_s << " s（vy=0, vyaw=0；10 Hz 重发）\n";
  log << "straight 允许范围: vx<=" << kMaxVxMps << " m/s, duration<="
      << kMaxDurationS << " s；实际速度与距离不作保证。\n";
  log << "pulse 允许范围: vx " << kMinVxMps << "-" << kMaxVxMps
      << " m/s, duration " << kPulseMinDurationS << "-" << kPulseMaxDurationS
      << " s；实际位移包含指令延迟和停车距离，不能按 vx*duration 估算。\n";
  log << "plan/goto 参数: vmax=" << config.goto_max_speed_mps
      << " m/s, approach=" << config.goto_approach_speed_mps
      << " m/s（approach 为首轮防失速候选，不是已验证的硬件最低速度）\n";
  log << "plan/goto 位置容差=" << kPositionToleranceM
      << " m（原生估计到达判据，不保证实际到点精度）；"
         "正常运动只发送 vx/vyaw，vy 固定为 0。\n";

  auto printHelp = [&]() {
    log << "命令: status | origin | straight | pulse vx duration | goto x y | "
           "plan forward left | yaw deg | stop | quit | help\n"
        << "  status        显示位置、航向、速度、状态时龄和当前任务\n"
        << "  origin        确认机器人静止并记录初始位置/航向；本轮只允许一次\n"
        << "  straight      用 --vx/--duration-s 做短时直走（当前机体系前方）\n"
        << "  pulse vx dur  短脉冲：vx " << kMinVxMps << "-" << kMaxVxMps
        << " m/s，duration " << kPulseMinDurationS << "-"
        << kPulseMaxDurationS << " s；到期立即 StopMove\n"
        << "  goto x y      到初始坐标系中的固定点，用路径+Pure Pursuit 跟踪\n"
        << "  plan f l      按当前机头方向指定相对目标（前 f、左 l），一次固定\n"
        << "  yaw deg       原地转到相对初始朝向的角度，单位 deg\n"
        << "  stop          终止当前任务并调用 StopMove\n"
        << "  quit          停止后退出\n"
        << "普通命令按 Enter 执行；Ctrl+C 立即中断，不需要 Enter。\n"
        << "正常运动只使用 vx/vyaw，vy 固定为 0；不再侧移。\n"
        << "运动中可输入 status/stop/quit；新的运动命令需先 stop。\n";
    if (config.control_source == ControlSource::Vins) {
      log << "vins 模式: plan/goto/yaw 使用 VINS；straight/pulse 仅 native。\n";
    }
  };

  log << "本地坐标: 起点=(0,0), 初始前方=+X, 左侧=+Y, 初始朝向=0°, "
         "左转为正。\n";
  log << "先让用户确认机器人静止、遥控在手，再输入 origin 记录起点。\n";
  printHelp();
  log << "命令> " << std::flush;

  auto printStatus = [&]() {
    refreshControl(runtime.now_s());
    log << "control_source=" << controlSourceName(config.control_source)
        << " control_valid=" << (control_valid ? "1" : "0")
        << " reason=" << control_reason << "\n";
    log << "task=" << taskName(task.type);
    if (task.type == TaskType::Goto || task.type == TaskType::Plan) {
      log << " target_local=(" << task.target_local_x << ","
          << task.target_local_y << ")";
      if (config.control_source == ControlSource::Native) {
        log << " target_native=(" << task.target_native_x << ","
            << task.target_native_y << ")";
      }
      log << " path_len=" << task.path.length()
          << " vmax=" << config.goto_max_speed_mps
          << " approach=" << config.goto_approach_speed_mps;
    } else if (task.type == TaskType::Yaw) {
      log << " target_local_yaw=" << task.target_local_yaw_deg
          << " deg target_yaw_raw=" << task.target_native_yaw;
    } else if (task.type == TaskType::Straight) {
      log << " duration=" << task.straight_duration_s
          << " s vx=" << task.straight_vx_mps << " m/s";
    } else if (task.type == TaskType::Pulse) {
      log << " vx=" << task.pulse_vx_mps
          << " m/s duration=" << task.pulse_duration_s << " s";
    }
    log << "\n";

    if (origin.set) {
      if (config.control_source == ControlSource::Vins) {
        log << "origin=vins_nav (native origin fields unused)\n";
      } else {
        log << "origin=set native=(" << origin.native_x << ","
            << origin.native_y << ") yaw_raw=" << origin.native_yaw << "\n";
      }
      if (control_local_available) {
        log << "control local pos=(" << control_local.x << ","
            << control_local.y << ") heading="
            << (control_local.heading * 180.0 / kPi) << " deg\n";
      } else {
        log << "control local=unavailable\n";
      }
    } else {
      log << "origin=unset\n";
    }

    if (config.control_source == ControlSource::Vins) {
      log << "vins seq=" << vins_state.seq
          << " stamp=" << vins_state.stamp_s
          << " age=" << vinsAgeS(vins_state, runtime.now_s()) << " s"
          << " valid=" << (vins_state.valid ? "1" : "0")
          << " reason=" << vinsReasonForDisplay(vins_state, runtime.now_s(),
                                                config.vins_timeout_s)
          << " vx=" << vins_state.vx << " vy=" << vins_state.vy
          << " wz=" << vins_state.wz
          << " measured=" << (control_measured ? "1" : "0")
          << " measured_override=" << (unmeasured_override ? "1" : "0")
          << "\n";
    }

    if (state.received) {
      log << "native pos=(" << state.position[0] << ","
          << state.position[1] << "," << state.position[2] << ") vel=("
          << state.velocity[0] << "," << state.velocity[1] << ","
          << state.velocity[2] << ") yaw_raw=" << state.yaw_rad << "\n";
    } else {
      log << "native=not received\n";
    }
    log << "native state seq=" << state.seq << " age=" << state.age_s
        << " s\n";
    log << "stop_failed=" << (session_state.stop_failed ? "YES" : "no") << "\n";
    if (session_state.stop_failed) {
      log << "!! 停止请求此前失败；本会话只允许 status、stop、quit。"
             "请现场确认机器人已停止；不会自动重试或恢复运动。\n";
    }
  };

  auto performStop = [&](TaskEndReason reason, bool show_prompt) -> bool {
    const bool was_failed = session_state.stop_failed;
    const bool had_task = task.type != TaskType::None;
    const Task ended_task = task;
    if (had_task) {
      writeEvent("task_end", taskEndReasonName(reason), &ended_task);
    }
    task = Task{};
    writeEvent("stop_request", taskEndReasonName(reason));

    int32_t stop_return = -1;
    bool stop_exception = false;
    std::string stop_error;
    try {
      stop_return = backend.stopMove();
    } catch (const std::exception& error) {
      stop_exception = true;
      stop_return = -1;
      stop_error = error.what();
    } catch (...) {
      stop_exception = true;
      stop_return = -1;
      stop_error = "unknown exception in StopMove";
    }
    if (stop_return != 0 && stop_error.empty()) {
      stop_error = backend.lastError();
      if (stop_error.empty()) {
        stop_error = "StopMove returned nonzero";
      }
    }
    const bool stop_ok = !stop_exception && stop_return == 0;
    if (!stop_ok) {
      session_state.stop_failed = true;
    }
    state = backend.snapshot();
    writeEvent("stop_result", stop_error, nullptr, true, stop_return);

    log << "停止流程: reason=" << taskEndReasonName(reason)
        << " StopMove返回=" << stop_return;
    if (stop_exception) {
      log << " (调用异常)";
    }
    if (!stop_error.empty()) {
      log << " detail=" << stop_error;
    }
    log << "\n";
    log << "StopMove 返回/程序状态不证明机器人已实际停住；请现场确认。\n";
    if (!stop_ok) {
      log << "!! StopMove 失败：请立即用遥控器或现场方式停止机器人，"
             "并保持安全距离。\n";
      log << "!! 停止失败状态已生效：本会话拒绝运动命令，"
             "只允许 status、stop、quit；不会自动重试或恢复运动。\n";
    } else if (was_failed) {
      log << "StopMove 返回 0；但本会话的停止失败状态保持，"
             "运动命令仍被拒绝。请现场确认机器人已停止；"
             "需要继续运动请 quit 后重新启动。\n";
    }
    if (show_prompt) {
      log << "命令> " << std::flush;
    }
    return stop_ok;
  };

  auto canStart = [&]() -> bool {
    if (session_state.stop_failed) {
      log << "!! 停止请求此前失败；本会话拒绝运动命令。"
             "请现场确认机器人已停止；只允许 status、stop、quit。\n";
      writeEvent("terminal_rejected", "stop_failed");
      return false;
    }
    if (task.type != TaskType::None) {
      log << "当前已有任务；先输入 stop。\n";
      writeEvent("terminal_rejected", "task_active");
      return false;
    }
    if (!origin.set) {
      log << "尚未记录起点；先输入 origin。\n";
      writeEvent("terminal_rejected", "origin_unset");
      return false;
    }
    refreshControl(runtime.now_s());
    if (!control_valid) {
      if (config.control_source == ControlSource::Vins) {
        log << "VINS 状态不可用（" << control_reason
            << "），暂不能开始运动。\n";
        writeEvent("terminal_rejected", "vins_invalid:" + control_reason);
      } else {
        log << "原生状态不新鲜，暂不能开始运动。\n";
        writeEvent("terminal_rejected", "state_stale");
      }
      return false;
    }
    return true;
  };

  auto startTask = [&](const Task& new_task, const std::string& detail) {
    task = new_task;
    task.start_s = runtime.now_s();
    previous_command_speed_mps = 0.0;
    if (config.control_source == ControlSource::Vins &&
        control_state.received) {
      motion_consistency.reset(
          task.start_s,
          {control_state.position[0], control_state.position[1]});
    }
    next_control = task.start_s;
    writeEvent("task_start", detail, &task);
    log << "任务开始: " << taskName(task.type) << " " << detail << "\n";
    log << "运动中可输入 status/stop/quit；新的运动命令需先 stop。\n";
    log << "运动中命令> " << std::flush;
  };

  auto purePursuitRadius = [&]() {
    return std::max(1e-3,
                    config.goto_max_speed_mps /
                        std::max(kMaxYawRateRadps, 1e-6));
  };

  // Builds a fixed-local-target task with a one-time path. The path starts at
  // the base_link pose at command time and ends at (target_x, target_y).
  auto makeNavigationTask = [&](TaskType type, double target_x,
                                double target_y) {
    const LocalPose local = toLocalPose(control_state, origin);
    const double dx = target_x - local.x;
    const double dy = target_y - local.y;
    const double distance = std::hypot(dx, dy);
    const double goal_yaw =
        distance > 1e-9 ? std::atan2(dy, dx) : local.heading;

    Task new_task;
    new_task.type = type;
    new_task.target_local_x = target_x;
    new_task.target_local_y = target_y;
    new_task.target_local_yaw_deg = goal_yaw * 180.0 / kPi;
    localToNative(origin, target_x, target_y, &new_task.target_native_x,
                  &new_task.target_native_y);
    new_task.target_native_yaw = wrapPi(origin.native_yaw + goal_yaw);
    new_task.timeout_s = kGotoTimeoutS;

    const go2_nav::Pose2D start{local.x, local.y, local.heading};
    const go2_nav::Pose2D goal{target_x, target_y, goal_yaw};
    new_task.path = go2_nav::makePath(start, goal, purePursuitRadius());
    return new_task;
  };

  auto startNavigationTask = [&](TaskType type, double target_x,
                                 double target_y, const std::string& prefix) {
    Task new_task = makeNavigationTask(type, target_x, target_y);
    const LocalPose local = toLocalPose(control_state, origin);
    const double distance = std::hypot(target_x - local.x,
                                       target_y - local.y);
    if (distance <= kPositionToleranceM || new_task.path.empty()) {
      log << "目标已在 " << kPositionToleranceM
          << " m 容差内或路径为空；按到达处理，不发送 Move。\n";
    }
    startTask(new_task,
              prefix + "local=(" + number(target_x, 3) + "," +
                  number(target_y, 3) + ") native=(" +
                  number(new_task.target_native_x, 3) + "," +
                  number(new_task.target_native_y, 3) + ") path_len=" +
                  number(new_task.path.length(), 3) + " radius=" +
                  number(purePursuitRadius(), 3) + " vy=0");
  };

  auto handleOrigin = [&]() {
    if (task.type != TaskType::None) {
      log << "任务中不能记录起点；先 stop。\n";
      writeEvent("terminal_rejected", "task_active");
      return;
    }
    if (origin.set) {
      log << "本轮不支持运行中重设原点；需要重新记录请 quit 后重启。\n";
      writeEvent("terminal_rejected", "origin_already_set");
      return;
    }

    if (config.control_source == ControlSource::Vins) {
      std::string error;
      bool measured = false;
      const bool ok =
          vins_client != nullptr &&
          vins_client->requestOrigin(&error, config.vins_origin_timeout_s,
                                     &measured);
      control_measured = measured;
      if (!ok) {
        if (error.empty()) {
          error = "no VINS bridge client";
        }
        log << "VINS origin 失败: " << error << "；不进入运动。\n";
        writeEvent("terminal_rejected", "vins_origin_failed:" + error);
        return;
      }
      if (!measured && !config.allow_unmeasured_extrinsic) {
        log << "T_base_cam measured=false；默认拒绝 VINS origin 和运动。"
               "仅机制验证可加 --allow-unmeasured-extrinsic；"
               "仍不得用于 5 cm 验收。\n";
        writeEvent("terminal_rejected", "unmeasured_extrinsic");
        return;
      }
      origin.set = true;
      origin.native_x = 0.0;
      origin.native_y = 0.0;
      origin.native_yaw = 0.0;
      unmeasured_override = !measured;
      refreshControl(runtime.now_s());
      writeEvent("origin_set", measured ? "vins_nav measured=1"
                                        : "vins_nav measured=0 OVERRIDE");
      log << "VINS origin 已建立; 控制来源=vins; measured="
          << (measured ? "1" : "0")
          << "; 等待 VINS STATE 后才允许 plan/goto/yaw。\n";
      if (!measured) {
        log << "!! 当前 T_base_cam 未实测；已显式允许未测量外参，"
               "只用于离线/机制验证，不得用于实机控制或 5 cm 验收。\n";
      }
      return;
    }

    state = backend.snapshot();
    if (!isFresh(state)) {
      log << "原生状态不新鲜，暂不能记录起点。\n";
      writeEvent("terminal_rejected", "state_stale");
      return;
    }
    origin.set = true;
    origin.native_x = state.position[0];
    origin.native_y = state.position[1];
    origin.native_yaw = state.yaw_rad;
    writeEvent("origin_set", "native_x=" + number(origin.native_x, 6) +
                                 " native_y=" + number(origin.native_y, 6) +
                                 " native_yaw_raw=" +
                                 number(origin.native_yaw, 6));
    log << "起点已记录: native x=" << origin.native_x
        << " y=" << origin.native_y << " yaw_raw=" << origin.native_yaw
        << "\n";
    log << "记录时 state seq=" << state.seq << " age=" << state.age_s
        << " s native velocity=(" << state.velocity[0] << ","
        << state.velocity[1] << "," << state.velocity[2] << ")\n";
    log << "本地坐标: 起点=(0,0), 初始前方=+X, 左侧=+Y, 初始朝向=0°, "
           "左转为正。\n";
    log << "后续用地面标记和卷尺核对实际误差；原生位置只是估计反馈。\n";
  };

  while (true) {
    if (runtime.interrupted()) {
      const bool stop_ok = performStop(TaskEndReason::Interrupted, false);
      const bool unresolved = session_state.stop_failed;
      writeEvent("end", unresolved ? "INTERRUPTED_STOP_FAILED"
                                   : "INTERRUPTED");
      if (unresolved) {
        log << "收到 Ctrl+C/SIGTERM；停止失败状态未解除，退出码 3。"
               "请现场确认机器人已停止。\n";
      } else {
        log << "收到 Ctrl+C/SIGTERM；已尝试停止并退出。\n";
      }
      return (stop_ok && !unresolved) ? 0 : 3;
    }

    Command command;
    if (runtime.poll_command(command)) {
      writeEvent("terminal", std::string(commandName(command.type)) + " | " +
                                command.raw);
      if (session_state.stop_failed &&
          command.type != CommandType::Status &&
          command.type != CommandType::Stop &&
          command.type != CommandType::Quit) {
        log << "!! 停止请求此前失败；本会话只允许 status、stop、quit。"
               "请现场确认机器人已停止；不会自动重试或恢复运动。\n";
        writeEvent("terminal_rejected", "stop_failed");
        log << "命令> " << std::flush;
        continue;
      }
      bool show_prompt = true;
      switch (command.type) {
        case CommandType::Invalid:
          log << "无法识别命令: " << command.raw << "\n";
          printHelp();
          writeEvent("terminal_rejected", "invalid");
          break;
        case CommandType::Help:
          printHelp();
          break;
        case CommandType::Status:
          printStatus();
          writeEvent("status",
                     session_state.stop_failed ? "stop_failed" : "");
          break;
        case CommandType::Origin:
          handleOrigin();
          break;
        case CommandType::Straight: {
          if (config.control_source == ControlSource::Vins) {
            log << "straight 只支持 --control-source native；"
                   "VINS 模式下未发送运动。\n";
            writeEvent("terminal_rejected", "vins_source_requires_native");
            break;
          }
          if (!canStart()) {
            break;
          }
          Task new_task;
          new_task.type = TaskType::Straight;
          new_task.straight_duration_s = config.duration_s;
          new_task.straight_vx_mps = config.vx_mps;
          startTask(new_task, "vx=" + number(config.vx_mps, 3) +
                                  " duration=" + number(config.duration_s, 3));
          show_prompt = false;
          break;
        }
        case CommandType::Pulse: {
          if (config.control_source == ControlSource::Vins) {
            log << "pulse 只支持 --control-source native；"
                   "VINS 模式下未发送运动。\n";
            writeEvent("terminal_rejected", "vins_source_requires_native");
            break;
          }
          if (!canStart()) {
            break;
          }
          if (command.pulse_vx_mps < kMinVxMps ||
              command.pulse_vx_mps > kMaxVxMps) {
            log << "pulse vx " << command.pulse_vx_mps << " m/s 超出范围 ["
                << kMinVxMps << ", " << kMaxVxMps
                << "]；未发送运动。\n";
            writeEvent("terminal_rejected", "pulse_vx_range");
            break;
          }
          if (command.pulse_duration_s < kPulseMinDurationS ||
              command.pulse_duration_s > kPulseMaxDurationS) {
            log << "pulse duration " << command.pulse_duration_s
                << " s 超出范围 [" << kPulseMinDurationS << ", "
                << kPulseMaxDurationS << "]；未发送运动。\n";
            writeEvent("terminal_rejected", "pulse_duration_range");
            break;
          }
          Task new_task;
          new_task.type = TaskType::Pulse;
          new_task.pulse_vx_mps = command.pulse_vx_mps;
          new_task.pulse_duration_s = command.pulse_duration_s;
          startTask(new_task,
                    "vx=" + number(command.pulse_vx_mps, 3) +
                        " duration=" + number(command.pulse_duration_s, 3) +
                        " vy=0 vyaw=0");
          show_prompt = false;
          break;
        }
        case CommandType::Goto: {
          if (!canStart()) {
            break;
          }
          const double distance = std::hypot(command.x, command.y);
          if (distance > kMaxGotoDistanceM) {
            log << "goto 目标距离 " << distance << " m 超过上限 "
                << kMaxGotoDistanceM << " m；未启动。\n";
            writeEvent("terminal_rejected", "goto_distance_cap");
            break;
          }
          startNavigationTask(TaskType::Goto, command.x, command.y,
                              "fixed_local=(" + number(command.x, 3) + "," +
                                  number(command.y, 3) + ") ");
          show_prompt = false;
          break;
        }
        case CommandType::Plan: {
          if (!canStart()) {
            break;
          }
          const double distance = std::hypot(command.x, command.y);
          if (distance > kMaxGotoDistanceM) {
            log << "plan 相对距离 " << distance << " m 超过上限 "
                << kMaxGotoDistanceM << " m；未启动。\n";
            writeEvent("terminal_rejected", "plan_distance_cap");
            break;
          }
          const LocalPose local = toLocalPose(control_state, origin);
          const double c = std::cos(local.heading);
          const double s = std::sin(local.heading);
          const double target_x = local.x + c * command.x - s * command.y;
          const double target_y = local.y + s * command.x + c * command.y;
          startNavigationTask(TaskType::Plan, target_x, target_y,
                              "relative=(" + number(command.x, 3) + "," +
                                  number(command.y, 3) + ") ");
          show_prompt = false;
          break;
        }
        case CommandType::Yaw: {
          if (!canStart()) {
            break;
          }
          const double raw_rad = command.angle_deg * kPi / 180.0;
          if (!std::isfinite(raw_rad)) {
            log << "yaw 角度过大；未启动。\n";
            writeEvent("terminal_rejected", "yaw_angle_invalid");
            break;
          }
          const double target_rad = wrapPi(raw_rad);
          const double target_deg = target_rad * 180.0 / kPi;
          Task new_task;
          new_task.type = TaskType::Yaw;
          new_task.target_local_yaw_deg = target_deg;
          new_task.target_native_yaw =
              wrapPi(origin.native_yaw + target_rad);
          new_task.timeout_s = kYawTimeoutS;
          startTask(new_task,
                    "local_deg=" + number(target_deg, 3) +
                        " target_yaw_raw=" +
                        number(new_task.target_native_yaw, 6));
          show_prompt = false;
          break;
        }
        case CommandType::Stop:
          performStop(TaskEndReason::Stopped, true);
          show_prompt = false;
          break;
        case CommandType::Quit: {
          const bool stop_ok = performStop(TaskEndReason::Quit, false);
          const bool unresolved = session_state.stop_failed;
          writeEvent("end",
                     unresolved ? "QUIT_STOP_FAILED" : "QUIT");
          if (unresolved) {
            log << "quit；停止失败状态未解除，退出码 3。"
                   "请现场确认机器人已停止。\n";
          } else {
            log << "quit；已尝试停止并退出。\n";
          }
          return (stop_ok && !unresolved) ? 0 : 3;
        }
      }
      if (show_prompt) {
        log << (task.type == TaskType::None ? "命令> " : "运动中命令> ")
            << std::flush;
      }
      continue;
    }

    if (task.type == TaskType::None) {
      // In vins mode, drain the bridge STATE socket even while idle. The
      // bridge streams STATE at 20 Hz; without an idle drain the kernel
      // receive buffer fills with stale packets and the first command after
      // a pause reads an old packet and reports STALE. snapshot() is
      // non-blocking: drain() returns on EAGAIN and keeps the latest STATE.
      if (config.control_source == ControlSource::Vins &&
          vins_client != nullptr) {
        vins_client->snapshot();
      }
      runtime.sleep_s(kCommandPollPeriodS);
      continue;
    }

    refreshControl(runtime.now_s());
    if (control_valid && config.control_source == ControlSource::Vins) {
      const double vins_speed = std::hypot(vins_state.vx, vins_state.vy);
      const double native_speed = isFresh(state)
          ? std::hypot(std::hypot(state.velocity[0], state.velocity[1]),
                       state.velocity[2])
          : vins_speed;
      const auto fault = motion_consistency.observe(
          runtime.now_s(), {control_state.position[0], control_state.position[1]},
          previous_command_speed_mps, native_speed, vins_speed);
      if (fault != MotionConsistencyFault::None) {
        control_valid = false;
        control_reason = fault == MotionConsistencyFault::
                             VinsDisplacementExceedsCommandBound
            ? "KINEMATIC_DISPLACEMENT"
            : "KINEMATIC_SPEED_DIVERGENCE";
        log << "运动一致性校验失败: " << control_reason
            << " command_distance=" << motion_consistency.command_distance_m()
            << " m vins_distance=" << motion_consistency.vins_distance_m()
            << " m；立即停止，不判 ARRIVED。\n";
      }
    }
    if (!control_valid) {
      if (config.control_source == ControlSource::Vins) {
        log << "VINS 状态不可用（" << control_reason
            << "）；立即停止，不回退原生位置。\n";
        performStop(TaskEndReason::VinsInvalid, true);
      } else {
        log << "任务中原生状态不新鲜: " << staleDetail(state) << "\n";
        performStop(TaskEndReason::StateStale, true);
      }
      continue;
    }

    const double elapsed = runtime.now_s() - task.start_s;
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    bool done = false;
    TaskEndReason end_reason = TaskEndReason::Arrived;

    switch (task.type) {
      case TaskType::Straight:
        if (elapsed + kScheduleEpsilonS >= task.straight_duration_s) {
          done = true;
          end_reason = TaskEndReason::Completed;
        } else {
          vx = task.straight_vx_mps;
        }
        break;
      case TaskType::Pulse:
        if (elapsed + kScheduleEpsilonS >= task.pulse_duration_s) {
          done = true;
          end_reason = TaskEndReason::Completed;
        } else {
          vx = task.pulse_vx_mps;
        }
        break;
      case TaskType::Goto:
      case TaskType::Plan: {
        const LocalPose local = toLocalPose(control_state, origin);
        const double dx = task.target_local_x - local.x;
        const double dy = task.target_local_y - local.y;
        const double distance = std::hypot(dx, dy);

        if (task.path.empty() || distance <= kPositionToleranceM) {
          if (task.path.empty() && distance > kPositionToleranceM) {
            log << "路径为空；按到达处理（位置误差 " << distance
                << " m），未发送运动。\n";
          }
          done = true;
          end_reason = TaskEndReason::Arrived;
        } else if (elapsed + kScheduleEpsilonS >= task.timeout_s) {
          done = true;
          end_reason = TaskEndReason::Timeout;
        } else {
          // Use the selected control source's velocity and yaw rate. In vins
          // mode this is the bridge's base_link twist; in native mode it is
          // the SportModeState velocity (still not an independent truth).
          go2_nav::BodyState body;
          body.stamp_s = runtime.now_s();
          body.position_m = go2_nav::Vec3{local.x, local.y, 0.0};
          body.yaw_rad = local.heading;
          body.velocity_mps = go2_nav::Vec2{control_state.velocity[0],
                                            control_state.velocity[1]};
          body.yaw_rate_radps = control_yaw_rate_radps;
          body.valid = control_valid;

          go2_nav::PurePursuitConfig track_config;
          track_config.max_speed_mps = config.goto_max_speed_mps;
          track_config.min_speed_mps = config.goto_approach_speed_mps;
          track_config.max_yaw_rate_radps = kMaxYawRateRadps;
          track_config.stop_distance_m = kPositionToleranceM;
          track_config.goal_tolerance_m = kPositionToleranceM;
          go2_nav::PurePursuit tracker(track_config);
          const go2_nav::VelocityCommand tracker_command =
              tracker.compute(body, task.path, task.tracker_state);

          if (tracker_command.stop_request) {
            done = true;
            end_reason = distance <= kPositionToleranceM
                             ? TaskEndReason::Arrived
                             : TaskEndReason::Timeout;
            if (end_reason == TaskEndReason::Timeout) {
              log << "Pure Pursuit 路径结束但位置误差 " << distance
                  << " m 超过容差，按未到达停止。\n";
            }
          } else {
            vx = tracker_command.vx_mps;
            vyaw = tracker_command.vyaw_radps;
          }
        }
        break;
      }
      case TaskType::Yaw: {
        const double error =
            wrapPi(task.target_native_yaw - control_state.yaw_rad);
        if (std::abs(error) <= kYawToleranceRad) {
          done = true;
          end_reason = TaskEndReason::Arrived;
        } else if (elapsed + kScheduleEpsilonS >= task.timeout_s) {
          done = true;
          end_reason = TaskEndReason::Timeout;
        } else {
          vyaw = std::clamp(kYawGain * error, -kMaxYawRateRadps,
                            kMaxYawRateRadps);
        }
        break;
      }
      case TaskType::None:
        break;
    }

    if (done) {
      log << "任务结束: " << taskName(task.type) << " "
          << taskEndReasonName(end_reason) << "\n";
      performStop(end_reason, true);
      continue;
    }

    // A1 boundary: normal motion only uses forward vx and yaw vyaw.
    vy = 0.0;

    const double control_time = runtime.now_s();
    if (control_time + kScheduleEpsilonS < next_control) {
      runtime.sleep_s(next_control - control_time);
      continue;
    }

    int32_t move_return = -1;
    std::string call_error;
    const double command_time = runtime.now_s();
    try {
      move_return = backend.move(static_cast<float>(vx), static_cast<float>(vy),
                                 static_cast<float>(vyaw));
    } catch (const std::exception& error) {
      call_error = error.what();
    } catch (...) {
      call_error = "unknown exception in Move";
    }
    if (move_return != 0 && call_error.empty()) {
      call_error = backend.lastError();
      if (call_error.empty()) {
        call_error = "Move returned nonzero";
      }
    }
    {
      RowContext context;
      context.state = &state;
      context.origin = &origin;
      context.task = &task;
      context.has_cmd = true;
      context.vx = vx;
      context.vy = vy;
      context.vyaw = vyaw;
      context.has_api = true;
      context.api = move_return;
      context.detail = call_error;
      context.vins_control = config.control_source == ControlSource::Vins;
      context.vins = context.vins_control ? &vins_state : nullptr;
      context.vins_valid = control_valid;
      context.vins_reason = control_reason;
      context.vins_measured = control_measured;
      context.measured_override = unmeasured_override;
      context.has_local = control_local_available;
      context.local = control_local;
      context.has_vins_age =
          context.vins_control && vins_state.received;
      context.vins_age_s =
          context.has_vins_age ? runtime.now_s() - vins_state.stamp_s : 0.0;
      writeRow(csv, command_time - session_t0, "cmd", context);
      csv.flush();
    }
    if (move_return != 0) {
      log << "Move 返回错误: " << call_error << "\n";
      performStop(TaskEndReason::MoveApiError, true);
      continue;
    }

    previous_command_speed_mps = std::hypot(vx, vy);

    next_control += kMovePeriodS;
    const double now_after_move = runtime.now_s();
    if (next_control < now_after_move) {
      next_control = now_after_move + kMovePeriodS;
    }
    const double wait_s = next_control - now_after_move;
    if (wait_s > 0.0) {
      runtime.sleep_s(wait_s);
    }
  }
}

}  // namespace go2_native_motion
