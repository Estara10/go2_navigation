#include "native_motion_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace g = go2_native_motion;

namespace {

int failures = 0;

void check(bool condition, const std::string& name) {
  if (!condition) {
    std::cerr << "FAIL " << name << "\n";
    ++failures;
  }
}

void checkNear(double actual, double expected, const std::string& name,
               double tolerance = 1e-6) {
  check(std::abs(actual - expected) <= tolerance, name);
}

std::size_t countOccurrences(const std::string& text,
                             const std::string& needle) {
  std::size_t count = 0;
  std::size_t position = 0;
  while ((position = text.find(needle, position)) != std::string::npos) {
    ++count;
    position += needle.size();
  }
  return count;
}

bool noCmdAfter(const std::string& csv, const std::string& marker) {
  const std::size_t position = csv.find(marker);
  if (position == std::string::npos) {
    return false;
  }
  return csv.find(",cmd,", position) == std::string::npos;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string current;
  bool quoted = false;
  for (const char c : line) {
    if (c == '"') {
      quoted = !quoted;
      current += c;
    } else if (c == ',' && !quoted) {
      fields.push_back(current);
      current.clear();
    } else {
      current += c;
    }
  }
  fields.push_back(current);
  return fields;
}

// cmd_vy_mps is column 21 after the pulse columns were added to the header.
bool cmdRowsHaveZeroVy(const std::string& csv) {
  std::istringstream stream(csv);
  std::string line;
  int cmd_rows = 0;
  bool all_zero = true;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 21 || fields[2] != "cmd") {
      continue;
    }
    ++cmd_rows;
    try {
      if (std::abs(std::stod(fields[21])) > 1e-12) {
        all_zero = false;
      }
    } catch (...) {
      all_zero = false;
    }
  }
  return cmd_rows > 0 && all_zero;
}

bool cmdRowsHaveTarget(const std::string& csv, double x, double y) {
  std::istringstream stream(csv);
  std::string line;
  int cmd_rows = 0;
  bool all_match = true;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 14 || fields[2] != "cmd") {
      continue;
    }
    ++cmd_rows;
    try {
      if (std::abs(std::stod(fields[9]) - x) > 1e-6 ||
          std::abs(std::stod(fields[10]) - y) > 1e-6) {
        all_match = false;
      }
    } catch (...) {
      all_match = false;
    }
  }
  return cmd_rows > 0 && all_match;
}

bool cmdRowsControlSource(const std::string& csv, const std::string& source) {
  std::istringstream stream(csv);
  std::string line;
  int cmd_rows = 0;
  bool all_match = true;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 30 || fields[2] != "cmd") {
      continue;
    }
    ++cmd_rows;
    if (fields[30] != source) {
      all_match = false;
    }
  }
  return cmd_rows > 0 && all_match;
}

bool cmdRowsVinsValid(const std::string& csv, const std::string& valid) {
  std::istringstream stream(csv);
  std::string line;
  int cmd_rows = 0;
  bool all_match = true;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 31 || fields[2] != "cmd") {
      continue;
    }
    ++cmd_rows;
    if (fields[31] != valid) {
      all_match = false;
    }
  }
  return cmd_rows > 0 && all_match;
}

bool cmdRowsVinsMeasured(const std::string& csv, const std::string& measured) {
  std::istringstream stream(csv);
  std::string line;
  int cmd_rows = 0;
  bool all_match = true;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 39 || fields[2] != "cmd") {
      continue;
    }
    ++cmd_rows;
    if (fields[39] != measured) {
      all_match = false;
    }
  }
  return cmd_rows > 0 && all_match;
}

bool cmdRowsMeasuredOverride(const std::string& csv,
                             const std::string& override_value) {
  std::istringstream stream(csv);
  std::string line;
  int cmd_rows = 0;
  bool all_match = true;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 40 || fields[2] != "cmd") {
      continue;
    }
    ++cmd_rows;
    if (fields[40] != override_value) {
      all_match = false;
    }
  }
  return cmd_rows > 0 && all_match;
}

bool cmdRowsVinsVelocity(const std::string& csv, double vx, double vy,
                         double wz) {
  std::istringstream stream(csv);
  std::string line;
  int cmd_rows = 0;
  bool all_match = true;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 38 || fields[2] != "cmd") {
      continue;
    }
    ++cmd_rows;
    try {
      if (std::abs(std::stod(fields[36]) - vx) > 1e-6 ||
          std::abs(std::stod(fields[37]) - vy) > 1e-6 ||
          std::abs(std::stod(fields[38]) - wz) > 1e-6) {
        all_match = false;
      }
    } catch (...) {
      all_match = false;
    }
  }
  return cmd_rows > 0 && all_match;
}

bool taskStartPulseFields(const std::string& csv, double* vx, double* duration) {
  std::istringstream stream(csv);
  std::string line;
  while (std::getline(stream, line)) {
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() <= 42 || fields[3] != "pulse" ||
        fields[2] != "event" || fields[42] != "task_start") {
      continue;
    }
    try {
      *vx = std::stod(fields[15]);
      *duration = std::stod(fields[16]);
      return true;
    } catch (...) {
      return false;
    }
  }
  return false;
}

class FakeBackend final : public g::Backend {
 public:
  FakeBackend() {
    state.received = true;
    state.finite = true;
    state.seq = 1;
    state.age_s = 0.01;
    state.position = {0.0, 0.0, 0.30};
    state.yaw_rad = 0.0;
  }

  g::StateSnapshot snapshot() override { return state; }

  int32_t move(float vx, float vy, float vyaw) override {
    ++move_calls;
    last_vx = vx;
    last_vy = vy;
    last_vyaw = vyaw;
    if (std::abs(vy) > 1e-12) {
      ++nonzero_vy_calls;
    }
    max_abs_vyaw = std::max(max_abs_vyaw, std::abs(static_cast<double>(vyaw)));
    if (stale_after_moves >= 0 && move_calls >= stale_after_moves) {
      state.age_s = g::kMaxStateAgeS + 0.2;
    }
    if (move_throws) {
      throw std::runtime_error("fake Move exception");
    }
    if (move_fail_at >= 0 && move_calls >= move_fail_at) {
      error = "fake Move return " + std::to_string(move_return);
      return move_return;
    }
    return 0;
  }

  int32_t stopMove() override {
    ++stop_calls;
    if (stop_throws) {
      throw std::runtime_error("fake StopMove exception");
    }
    return stop_return;
  }

  std::string lastError() override { return error; }

  g::StateSnapshot state;
  int32_t move_return = 0;
  int32_t stop_return = 0;
  int move_fail_at = -1;
  int stale_after_moves = -1;
  bool move_throws = false;
  bool stop_throws = false;
  int move_calls = 0;
  int stop_calls = 0;
  int nonzero_vy_calls = 0;
  double last_vx = 0.0;
  double last_vy = 0.0;
  double last_vyaw = 0.0;
  double max_abs_vyaw = 0.0;
  std::string error;
};

struct FakeClock {
  double now = 0.0;
};

class FakeVinsClient final : public g::VinsClient {
 public:
  explicit FakeVinsClient(FakeClock& clock) : clock_(clock) {}

  go2_nav::VinsWireState snapshot() override {
    if (auto_stamp && state.received) {
      state.stamp_s = clock_.now;
    }
    return state;
  }

  bool requestOrigin(std::string* error, double /*timeout_s*/,
                     bool* measured_out) override {
    ++origin_requests;
    if (!origin_ok) {
      if (error != nullptr) {
        *error = origin_error;
      }
      return false;
    }
    state.received = true;
    state.valid = true;
    state.reason = go2_nav::VinsReason::Ok;
    state.measured = measured;
    ++state.seq;
    state.stamp_s = clock_.now;
    state.x = 0.0;
    state.y = 0.0;
    state.z = 0.0;
    state.yaw = 0.0;
    if (measured_out != nullptr) {
      *measured_out = measured;
    }
    return true;
  }

  void requestPing() override { ++ping_requests; }

  void setState(double x, double y, double yaw, bool valid,
                go2_nav::VinsReason reason) {
    state.received = true;
    state.valid = valid;
    state.reason = reason;
    state.measured = measured;
    ++state.seq;
    state.stamp_s = clock_.now;
    state.x = x;
    state.y = y;
    state.z = 0.0;
    state.yaw = yaw;
  }

  FakeClock& clock_;
  go2_nav::VinsWireState state;
  bool origin_ok = true;
  bool auto_stamp = true;
  bool measured = true;
  std::string origin_error = "NO_VINS";
  int origin_requests = 0;
  int ping_requests = 0;
};

class ScriptedCommands {
 public:
  explicit ScriptedCommands(FakeClock& clock) : clock_(clock) {}

  void add(double at_s, const std::string& line,
           std::function<void()> on_deliver = {}) {
    commands_.push_back({at_s, line, std::move(on_deliver)});
  }

  bool poll(g::Command& command) {
    if (index_ >= commands_.size()) {
      return false;
    }
    const Entry& entry = commands_[index_];
    if (clock_.now + 1e-9 < entry.at_s) {
      return false;
    }
    if (entry.on_deliver) {
      entry.on_deliver();
    }
    command = g::parseCommand(entry.line);
    ++index_;
    return true;
  }

 private:
  struct Entry {
    double at_s;
    std::string line;
    std::function<void()> on_deliver;
  };

  FakeClock& clock_;
  std::vector<Entry> commands_;
  std::size_t index_ = 0;
};

struct Case {
  FakeBackend backend;
  FakeClock clock;
  FakeVinsClient vins_client{clock};
  ScriptedCommands script{clock};
  g::Config config;
  g::SessionState session_state;
  std::ostringstream log;
  std::ostringstream csv;
  int exit_code = 0;
  bool interrupted = false;
  double interrupt_at = std::numeric_limits<double>::infinity();
};

void prepare(Case& test_case) {
  test_case.config.csv_path = "selftest.csv";
  test_case.config.vx_mps = 0.10;
  test_case.config.vy_mps = 0.0;
  test_case.config.vyaw_radps = 0.0;
  test_case.config.duration_s = 2.0;
  g::writeCsvHeader(test_case.csv);
}

int run(Case& test_case) {
  g::Runtime runtime;
  runtime.now_s = [&test_case] { return test_case.clock.now; };
  runtime.sleep_s = [&test_case](double seconds) {
    test_case.clock.now += seconds;
  };
  runtime.interrupted = [&test_case] {
    return test_case.interrupted ||
           test_case.clock.now + 1e-9 >= test_case.interrupt_at;
  };
  runtime.poll_command = [&test_case](g::Command& command) {
    return test_case.script.poll(command);
  };
  test_case.exit_code = g::runInteractiveSession(
      test_case.config, test_case.backend, &test_case.vins_client, runtime,
      test_case.session_state, test_case.log, test_case.csv);
  return test_case.exit_code;
}

void checkNoLateral(const Case& test_case, const std::string& name) {
  check(test_case.backend.nonzero_vy_calls == 0,
        name + " backend nonzero vy");
  check(cmdRowsHaveZeroVy(test_case.csv.str()),
        name + " csv cmd_vy zero");
}

void testTransforms() {
  const int before = failures;
  g::Origin origin;
  origin.set = true;
  origin.native_x = 1.0;
  origin.native_y = 2.0;
  origin.native_yaw = g::kPi / 2.0;

  double native_x = 0.0;
  double native_y = 0.0;
  g::localToNative(origin, 1.0, 0.0, &native_x, &native_y);
  checkNear(native_x, 1.0, "local forward x at yaw 90");
  checkNear(native_y, 3.0, "local forward y at yaw 90");
  g::localToNative(origin, 0.0, 1.0, &native_x, &native_y);
  checkNear(native_x, 0.0, "local left x at yaw 90");
  checkNear(native_y, 2.0, "local left y at yaw 90");

  g::StateSnapshot state;
  state.received = true;
  state.finite = true;
  state.position = {1.0, 2.0, 0.30};
  state.yaw_rad = g::kPi / 2.0;
  g::LocalPose local = g::toLocalPose(state, origin);
  checkNear(local.x, 0.0, "local pose origin x");
  checkNear(local.y, 0.0, "local pose origin y");
  checkNear(local.heading, 0.0, "local pose origin heading");

  state.position = {1.0, 3.0, 0.30};
  local = g::toLocalPose(state, origin);
  checkNear(local.x, 1.0, "local pose forward x");
  checkNear(local.y, 0.0, "local pose forward y");

  state.position = {0.0, 2.0, 0.30};
  local = g::toLocalPose(state, origin);
  checkNear(local.x, 0.0, "local pose left x");
  checkNear(local.y, 1.0, "local pose left y");

  std::cout << (failures == before ? "PASS" : "FAIL") << " transforms\n";
}

void testWrapPi() {
  const int before = failures;
  const double deg = g::kPi / 180.0;
  checkNear(g::wrapPi(179.0 * deg - (-179.0 * deg)), -2.0 * deg,
            "wrap 179 to -179 is -2 deg");
  checkNear(g::wrapPi(-179.0 * deg - 179.0 * deg), 2.0 * deg,
            "wrap -179 to 179 is +2 deg");
  checkNear(g::wrapPi(181.0 * deg), -179.0 * deg, "wrap 181");
  checkNear(g::wrapPi(-181.0 * deg), 179.0 * deg, "wrap -181");
  checkNear(g::wrapPi(g::kPi), g::kPi, "wrap +pi");
  checkNear(g::wrapPi(-g::kPi), g::kPi, "wrap -pi deterministic +pi");
  checkNear(g::wrapPi(0.1), 0.1, "wrap small angle");
  std::cout << (failures == before ? "PASS" : "FAIL") << " wrap_pi\n";
}

void testParseCommand() {
  const int before = failures;
  check(g::parseCommand("status").type == g::CommandType::Status,
        "parse status");
  check(g::parseCommand("origin").type == g::CommandType::Origin,
        "parse origin");
  check(g::parseCommand("straight").type == g::CommandType::Straight,
        "parse straight");
  check(g::parseCommand("stop").type == g::CommandType::Stop, "parse stop");
  check(g::parseCommand("quit").type == g::CommandType::Quit, "parse quit");
  check(g::parseCommand("help").type == g::CommandType::Help, "parse help");

  g::Command command = g::parseCommand("  GOTO   1.5   -2.5 ");
  check(command.type == g::CommandType::Goto, "parse goto type");
  checkNear(command.x, 1.5, "parse goto x");
  checkNear(command.y, -2.5, "parse goto y");
  command = g::parseCommand("plan 1.5 -2.5");
  check(command.type == g::CommandType::Plan, "parse plan type");
  checkNear(command.x, 1.5, "parse plan forward");
  checkNear(command.y, -2.5, "parse plan left");
  command = g::parseCommand("pulse 0.20 0.30");
  check(command.type == g::CommandType::Pulse, "parse pulse type");
  checkNear(command.pulse_vx_mps, 0.20, "parse pulse vx");
  checkNear(command.pulse_duration_s, 0.30, "parse pulse duration");
  command = g::parseCommand("yaw 90");
  check(command.type == g::CommandType::Yaw, "parse yaw type");
  checkNear(command.angle_deg, 90.0, "parse yaw angle");

  check(g::parseCommand("goto 1").type == g::CommandType::Invalid,
        "parse goto missing arg");
  check(g::parseCommand("goto a b").type == g::CommandType::Invalid,
        "parse goto bad number");
  check(g::parseCommand("plan 1").type == g::CommandType::Invalid,
        "parse plan missing arg");
  check(g::parseCommand("plan a b").type == g::CommandType::Invalid,
        "parse plan bad number");
  check(g::parseCommand("pulse 0.2").type == g::CommandType::Invalid,
        "parse pulse missing arg");
  check(g::parseCommand("pulse a b").type == g::CommandType::Invalid,
        "parse pulse bad number");
  check(g::parseCommand("yaw a").type == g::CommandType::Invalid,
        "parse yaw bad number");
  check(g::parseCommand("unknown").type == g::CommandType::Invalid,
        "parse unknown");
  check(g::parseCommand("").type == g::CommandType::Invalid, "parse empty");
  std::cout << (failures == before ? "PASS" : "FAIL") << " parse_command\n";
}

void testStraightPreserved() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.vx_mps = 0.10;
  test_case.config.duration_s = 1.0;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(2.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 10, "straight 10 Move calls");
  check(test_case.backend.stop_calls == 2, "straight stop + quit stop");
  checkNear(test_case.backend.last_vx, 0.10, "straight last vx");
  checkNear(test_case.backend.last_vy, 0.0, "straight last vy");
  checkNear(test_case.backend.last_vyaw, 0.0, "straight last vyaw");
  check(test_case.exit_code == 0, "straight quit exit 0");
  check(csv.find(",task_start,") != std::string::npos,
        "straight CSV task_start");
  check(csv.find("COMPLETED") != std::string::npos,
        "straight CSV completed");
  check(csv.find("QUIT") != std::string::npos, "straight CSV quit");
  check(noCmdAfter(csv, "stop_request"), "straight no cmd after stop");
  checkNoLateral(test_case, "straight");
  std::cout << (failures == before ? "PASS" : "FAIL") << " straight_preserved\n";
}

void testGotoForwardAndStop() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.state.position = {1.0, 2.0, 0.30};
  test_case.backend.state.yaw_rad = g::kPi / 2.0;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 5, "goto stop after 5 Move calls");
  checkNear(test_case.backend.last_vx, 0.30, "goto forward vx");
  checkNear(test_case.backend.last_vy, 0.0, "goto forward vy");
  checkNear(test_case.backend.last_vyaw, 0.0, "goto forward vyaw");
  check(csv.find("STOPPED") != std::string::npos, "goto CSV stopped");
  check(noCmdAfter(csv, "stop_request"), "goto no cmd after stop");
  checkNoLateral(test_case, "goto_forward");
  std::cout << (failures == before ? "PASS" : "FAIL") << " goto_forward_stop\n";
}

void testGotoLeft() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 0 1");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  checkNear(test_case.backend.last_vy, 0.0, "goto left vy is zero");
  check(test_case.backend.max_abs_vyaw > 0.0,
        "goto left turns instead of lateral motion");
  checkNoLateral(test_case, "goto_left");
  std::cout << (failures == before ? "PASS" : "FAIL") << " goto_left\n";
}

void testGotoApproachFloor() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 0.2 0");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  checkNear(test_case.backend.last_vx, 0.30,
            "goto approach floor keeps vx at 0.30");
  checkNear(test_case.backend.last_vy, 0.0, "goto approach floor vy");
  checkNoLateral(test_case, "goto_approach_floor");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " goto_approach_floor\n";
}

void testPulseBoundsAndTiming() {
  const int before = failures;

  // 0.1 s pulse: one Move at t=0.1, then StopMove.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "pulse 0.20 0.1");
    test_case.script.add(1.0, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.backend.move_calls == 1, "pulse 0.1 s sends one Move");
    checkNear(test_case.backend.last_vx, 0.20, "pulse 0.1 s vx");
    checkNear(test_case.backend.last_vy, 0.0, "pulse 0.1 s vy");
    checkNear(test_case.backend.last_vyaw, 0.0, "pulse 0.1 s vyaw");
    check(test_case.backend.stop_calls == 2, "pulse 0.1 s stop + quit");
    check(csv.find(",pulse,") != std::string::npos, "pulse CSV task");
    check(csv.find("COMPLETED") != std::string::npos,
          "pulse CSV completed");
    check(noCmdAfter(csv, "stop_request"), "pulse no cmd after stop");
    double vx = 0.0;
    double duration = 0.0;
    check(taskStartPulseFields(csv, &vx, &duration),
          "pulse CSV start fields present");
    checkNear(vx, 0.20, "pulse CSV vx field");
    checkNear(duration, 0.1, "pulse CSV duration field");
    checkNoLateral(test_case, "pulse_0.1");
  }

  // 0.5 s pulse: five 10 Hz Move calls, then StopMove.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "pulse 0.20 0.5");
    test_case.script.add(2.0, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 5, "pulse 0.5 s sends five Moves");
    checkNear(test_case.backend.last_vx, 0.20, "pulse 0.5 s vx");
    check(test_case.backend.stop_calls == 2, "pulse 0.5 s stop + quit");
    checkNoLateral(test_case, "pulse_0.5");
  }

  // Bounds are inclusive; values outside are rejected without a Move.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "pulse 0.05 0.1");
    test_case.script.add(0.6, "stop");
    test_case.script.add(1.0, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 1, "pulse lower bounds accepted");
    checkNear(test_case.backend.last_vx, 0.05, "pulse lower vx");
    checkNoLateral(test_case, "pulse_lower_bounds");
  }
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "pulse 0.50 0.5");
    test_case.script.add(1.0, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 5, "pulse upper bounds accepted");
    checkNear(test_case.backend.last_vx, 0.50, "pulse upper vx");
    checkNoLateral(test_case, "pulse_upper_bounds");
  }

  auto run_rejected = [&](const std::string& command,
                          const std::string& marker) {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, command);
    test_case.script.add(0.6, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 0,
          command + " rejected without Move");
    check(test_case.csv.str().find(marker) != std::string::npos,
          command + " rejection marker");
  };
  run_rejected("pulse 0.049 0.1", "pulse_vx_range");
  run_rejected("pulse 0.501 0.1", "pulse_vx_range");
  run_rejected("pulse 0.20 0.099", "pulse_duration_range");
  run_rejected("pulse 0.20 0.501", "pulse_duration_range");

  // Pulse requires origin and a fresh state.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.1, "pulse 0.20 0.1");
    test_case.script.add(0.6, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 0, "pulse without origin rejected");
    check(test_case.csv.str().find("origin_unset") != std::string::npos,
          "pulse without origin marker");
  }
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "pulse 0.20 0.1", [&test_case] {
      test_case.backend.state.age_s = g::kMaxStateAgeS + 0.1;
    });
    test_case.script.add(0.6, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 0, "pulse stale state rejected");
    check(test_case.csv.str().find("state_stale") != std::string::npos,
          "pulse stale state marker");
  }

  // A failed StopMove latches and rejects the next pulse.
  {
    Case test_case;
    prepare(test_case);
    test_case.backend.stop_return = 3104;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "pulse 0.20 0.1");
    test_case.script.add(1.0, "pulse 0.20 0.1");
    test_case.script.add(1.5, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.backend.move_calls == 1,
          "pulse stop failure latch no extra Move");
    check(test_case.session_state.stop_failed,
          "pulse stop failure latch state");
    check(test_case.exit_code == 3, "pulse stop failure exit 3");
    check(csv.find("terminal_rejected,stop_failed") != std::string::npos,
          "pulse stop failure CSV rejection");
    check(csv.find("QUIT_STOP_FAILED") != std::string::npos,
          "pulse stop failure end event");
    checkNoLateral(test_case, "pulse_stop_failure");
  }

  std::cout << (failures == before ? "PASS" : "FAIL")
            << " pulse_bounds_timing\n";
}

void testPlanAndGotoNavigation() {
  const int before = failures;

  // plan target is fixed at command time; changing the current heading later
  // must not move the stored target.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 1 0");
    test_case.script.add(0.2, "status", [&test_case] {
      test_case.backend.state.yaw_rad = g::kPi / 2.0;
    });
    test_case.script.add(0.6, "stop");
    test_case.script.add(1.0, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(cmdRowsHaveTarget(csv, 1.0, 0.0),
          "plan target stays fixed after turn");
    check(noCmdAfter(csv, "stop_request"), "plan no cmd after stop");
    checkNoLateral(test_case, "plan_target_fixed");
  }

  // goto keeps its initial-origin fixed-point semantics but uses the same
  // path + Pure Pursuit control law.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "goto 1 0");
    test_case.script.add(0.2, "status", [&test_case] {
      test_case.backend.state.yaw_rad = g::kPi / 2.0;
    });
    test_case.script.add(0.6, "stop");
    test_case.script.add(1.0, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(cmdRowsHaveTarget(csv, 1.0, 0.0),
          "goto target stays fixed after turn");
    checkNoLateral(test_case, "goto_target_fixed");
  }

  // A side target must turn and move forward, never translate sideways.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 0 1");
    test_case.script.add(0.6, "stop");
    test_case.script.add(1.0, "quit");
    run(test_case);
    check(test_case.backend.move_calls > 0, "plan side sends Moves");
    checkNear(test_case.backend.last_vy, 0.0, "plan side last vy zero");
    check(test_case.backend.max_abs_vyaw > 0.0,
          "plan side uses yaw instead of lateral motion");
    checkNoLateral(test_case, "plan_side");
  }

  // Target already inside tolerance: arrival without a Move.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 0.01 0");
    test_case.script.add(0.5, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.backend.move_calls == 0,
          "plan inside tolerance sends no Move");
    check(test_case.backend.stop_calls == 2,
          "plan inside tolerance arrival + quit stop");
    check(csv.find("ARRIVED") != std::string::npos,
          "plan inside tolerance ARRIVED");
  }

  // Distance caps reject before a task starts.
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 3.1 0");
    test_case.script.add(0.6, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 0, "plan distance cap no Move");
    check(test_case.csv.str().find("plan_distance_cap") != std::string::npos,
          "plan distance cap marker");
  }
  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "goto 3.1 0");
    test_case.script.add(0.6, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 0, "goto distance cap no Move");
    check(test_case.csv.str().find("goto_distance_cap") != std::string::npos,
          "goto distance cap marker");
  }

  std::cout << (failures == before ? "PASS" : "FAIL")
            << " plan_goto_navigation\n";
}

void testPlanTimeoutAndStateStale() {
  const int before = failures;

  {
    Case test_case;
    prepare(test_case);
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 1 0");
    test_case.script.add(31.0, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.backend.move_calls > 250, "plan timeout many Moves");
    check(test_case.backend.stop_calls == 2, "plan timeout stop + quit");
    check(csv.find("TIMEOUT") != std::string::npos, "plan timeout CSV");
    checkNoLateral(test_case, "plan_timeout");
  }

  {
    Case test_case;
    prepare(test_case);
    test_case.backend.stale_after_moves = 1;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 1 0");
    test_case.script.add(1.0, "quit");
    run(test_case);
    check(test_case.csv.str().find("STATE_STALE") != std::string::npos,
          "plan state stale CSV");
    check(test_case.backend.stop_calls == 2, "plan state stale stop + quit");
    checkNoLateral(test_case, "plan_state_stale");
  }

  std::cout << (failures == before ? "PASS" : "FAIL")
            << " plan_timeout_state_stale\n";
}

void testVinsControlSource() {
  const int before = failures;

  // plan uses VINS local state, not the (deliberately different) native state.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.backend.state.position = {5.0, 5.0, 0.30};
    test_case.backend.state.yaw_rad = 1.0;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 0.5 0", [&test_case] {
      test_case.vins_client.state.vx = 0.15;
      test_case.vins_client.state.vy = 0.05;
      test_case.vins_client.state.wz = 0.25;
    });
    test_case.script.add(0.6, "stop");
    test_case.script.add(1.0, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.vins_client.origin_requests == 1,
          "vins origin requested once");
    check(test_case.backend.move_calls > 0, "vins plan sends Moves");
    check(cmdRowsHaveTarget(csv, 0.5, 0.0),
          "vins plan target uses VINS local frame");
    check(cmdRowsVinsVelocity(csv, 0.15, 0.05, 0.25),
          "vins plan CSV velocity fields");
    check(cmdRowsControlSource(csv, "vins"),
          "vins plan CSV control source");
    check(cmdRowsVinsValid(csv, "1"), "vins plan CSV valid field");
    check(cmdRowsVinsMeasured(csv, "1"),
          "vins plan CSV measured field");
    check(cmdRowsMeasuredOverride(csv, "0"),
          "vins plan CSV override field");
    checkNear(test_case.backend.last_vy, 0.0, "vins plan last vy zero");
    checkNoLateral(test_case, "vins_plan");
    check(csv.find("VINS_INVALID") == std::string::npos,
          "vins plan not invalid");
  }

  // Origin failure must block motion.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.vins_client.origin_ok = false;
    test_case.vins_client.origin_error = "NO_VINS";
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 0.5 0");
    test_case.script.add(0.6, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.vins_client.origin_requests == 1,
          "vins origin failure requested once");
    check(test_case.backend.move_calls == 0,
          "vins origin failure no Move");
    check(csv.find("vins_origin_failed:NO_VINS") != std::string::npos,
          "vins origin failure marker");
  }

  // valid=0 stops an active plan immediately and does not fall back.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 1 0");
    test_case.script.add(0.2, "status", [&test_case] {
      test_case.vins_client.setState(0.0, 0.0, 0.0, false,
                                     go2_nav::VinsReason::Stale);
    });
    test_case.script.add(0.6, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.backend.move_calls == 1,
          "vins invalid stops after one Move");
    check(csv.find("VINS_INVALID") != std::string::npos,
          "vins invalid task end");
    check(csv.find("STALE") != std::string::npos,
          "vins invalid reason recorded");
    check(noCmdAfter(csv, "stop_request"), "vins invalid no cmd after stop");
    check(test_case.backend.stop_calls == 2,
          "vins invalid stop + quit");
  }

  // A stale timestamp (packet timeout) stops even if the last bridge state
  // still said valid.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.config.vins_timeout_s = 0.3;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 1 0");
    test_case.script.add(0.2, "status", [&test_case] {
      test_case.vins_client.auto_stamp = false;
      test_case.vins_client.state.received = true;
      test_case.vins_client.state.valid = true;
      test_case.vins_client.state.reason = go2_nav::VinsReason::Ok;
      test_case.vins_client.state.stamp_s = test_case.clock.now - 1.0;
    });
    test_case.script.add(0.6, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.backend.move_calls == 1,
          "vins timeout stops after one Move");
    check(csv.find("VINS_INVALID") != std::string::npos,
          "vins timeout task end");
    check(csv.find("STALE") != std::string::npos,
          "vins timeout reason recorded");
    check(noCmdAfter(csv, "stop_request"), "vins timeout no cmd after stop");
  }

  // straight and pulse are native-source commands; vins mode rejects them.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "straight");
    test_case.script.add(0.2, "pulse 0.20 0.1");
    test_case.script.add(0.6, "quit");
    run(test_case);
    check(test_case.backend.move_calls == 0,
          "vins rejects straight/pulse without Move");
    check(countOccurrences(test_case.csv.str(),
                           "vins_source_requires_native") == 2,
          "vins rejects straight/pulse markers");
  }

  // yaw uses VINS heading in vins mode.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "yaw 90");
    test_case.script.add(0.6, "stop");
    test_case.script.add(1.0, "quit");
    run(test_case);
    check(test_case.backend.move_calls > 0, "vins yaw sends Moves");
    check(test_case.backend.max_abs_vyaw > 0.0, "vins yaw turns");
    checkNear(test_case.backend.last_vy, 0.0, "vins yaw vy zero");
    check(cmdRowsControlSource(test_case.csv.str(), "vins"),
          "vins yaw CSV control source");
    checkNoLateral(test_case, "vins_yaw");
  }

  // measured=false is rejected by default.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.vins_client.measured = false;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 0.5 0");
    test_case.script.add(0.6, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.vins_client.origin_requests == 1,
          "unmeasured origin requested once");
    check(test_case.backend.move_calls == 0,
          "unmeasured origin rejected without Move");
    check(csv.find("unmeasured_extrinsic") != std::string::npos,
          "unmeasured origin rejection marker");
    check(csv.find("vins_origin_failed") == std::string::npos,
          "unmeasured origin is not a bridge failure");
  }

  // Explicit override allows origin/plan but marks measured=0 and override=1.
  {
    Case test_case;
    prepare(test_case);
    test_case.config.control_source = g::ControlSource::Vins;
    test_case.config.allow_unmeasured_extrinsic = true;
    test_case.vins_client.measured = false;
    test_case.script.add(0.0, "origin");
    test_case.script.add(0.1, "plan 0.5 0");
    test_case.script.add(0.6, "stop");
    test_case.script.add(1.0, "quit");
    run(test_case);
    const std::string csv = test_case.csv.str();
    check(test_case.backend.move_calls > 0,
          "unmeasured override allows Moves");
    check(cmdRowsVinsMeasured(csv, "0"),
          "unmeasured override CSV measured=0");
    check(cmdRowsMeasuredOverride(csv, "1"),
          "unmeasured override CSV override=1");
    check(csv.find("measured=0 OVERRIDE") != std::string::npos,
          "unmeasured override origin marker");
    checkNoLateral(test_case, "unmeasured_override");
  }

  std::cout << (failures == before ? "PASS" : "FAIL")
            << " vins_control_source\n";
}

void testYawWrap() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "yaw 190", [&test_case] {
    test_case.backend.state.yaw_rad = 170.0 * g::kPi / 180.0;
  });
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  check(test_case.backend.last_vyaw > 0.0, "yaw wrap turns left");
  checkNear(test_case.backend.last_vx, 0.0, "yaw wrap vx zero");
  checkNear(test_case.backend.last_vy, 0.0, "yaw wrap vy zero");
  std::cout << (failures == before ? "PASS" : "FAIL") << " yaw_wrap\n";
}

void testArrivalStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0", [&test_case] {
    test_case.backend.state.position = {0.94, 0.0, 0.30};
  });
  test_case.script.add(0.2, "status", [&test_case] {
    test_case.backend.state.position = {0.96, 0.0, 0.30};
  });
  test_case.script.add(0.5, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 1,
        "6 cm remaining continues; 4 cm remaining stops");
  check(test_case.backend.stop_calls == 2, "arrival stop + quit stop");
  check(csv.find("ARRIVED") != std::string::npos, "arrival CSV ARRIVED");
  check(countOccurrences(csv, ",cmd,") == 1, "arrival one cmd row");
  check(noCmdAfter(csv, "stop_request"), "arrival no cmd after stop");
  checkNoLateral(test_case, "arrival");
  std::cout << (failures == before ? "PASS" : "FAIL") << " arrival_stop\n";
}

void testTimeoutStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(31.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls > 250, "timeout many Move calls");
  check(test_case.backend.stop_calls == 2, "timeout stop + quit stop");
  check(csv.find("TIMEOUT") != std::string::npos, "timeout CSV TIMEOUT");
  checkNoLateral(test_case, "timeout");
  std::cout << (failures == before ? "PASS" : "FAIL") << " timeout_stop\n";
}

void testNewMotionRejected() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(0.5, "yaw 90");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(csv.find("task_active") != std::string::npos,
        "new motion rejected while task active");
  check(countOccurrences(csv, ",task_start,") == 1,
        "only one task_start");
  checkNear(test_case.backend.last_vyaw, 0.0,
            "rejected yaw did not change command");
  std::cout << (failures == before ? "PASS" : "FAIL") << " new_motion_rejected\n";
}

void testOriginRules() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "goto 1 0");
  test_case.script.add(0.1, "origin");
  test_case.script.add(0.2, "origin");
  test_case.script.add(0.5, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 0, "origin rules no Move");
  check(csv.find("origin_unset") != std::string::npos,
        "origin rules unset rejected");
  check(csv.find("origin_already_set") != std::string::npos,
        "origin rules re-zero rejected");
  check(countOccurrences(csv, "origin_set") == 1,
        "origin rules one origin_set");
  std::cout << (failures == before ? "PASS" : "FAIL") << " origin_rules\n";
}

void testMoveErrorStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.move_fail_at = 1;
  test_case.backend.move_return = 2311;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 1, "move error one Move");
  check(test_case.backend.stop_calls == 2, "move error stop + quit stop");
  check(csv.find("MOVE_API_ERROR") != std::string::npos,
        "move error CSV reason");
  check(csv.find("2311") != std::string::npos, "move error CSV return");
  std::cout << (failures == before ? "PASS" : "FAIL") << " move_error_stop\n";
}

void testStateStaleStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.stale_after_moves = 1;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(csv.find("STATE_STALE") != std::string::npos,
        "stale CSV reason");
  check(test_case.backend.stop_calls == 2, "stale stop + quit stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " state_stale_stop\n";
}

void testStopFailureExit() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.duration_s = 1.0;
  test_case.backend.stop_return = 3104;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(3.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(csv.find("COMPLETED") != std::string::npos,
        "stop failure completed task");
  check(csv.find("3104") != std::string::npos, "stop failure CSV return");
  check(test_case.exit_code == 3, "stop failure exit 3");
  std::cout << (failures == before ? "PASS" : "FAIL") << " stop_failure\n";
}

void testStopFailureLatch() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.duration_s = 1.0;
  test_case.backend.stop_return = 3104;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(1.5, "status");
  test_case.script.add(1.6, "goto 1 0");
  test_case.script.add(1.7, "yaw 90");
  test_case.script.add(1.8, "stop", [&test_case] {
    test_case.backend.stop_return = 0;
  });
  test_case.script.add(1.9, "straight");
  test_case.script.add(2.2, "goto 1 0");
  test_case.script.add(2.5, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  const std::string log = test_case.log.str();
  check(test_case.backend.move_calls == 10,
        "stop failure latch no extra Move");
  check(test_case.session_state.stop_failed,
        "stop failure latch state remains");
  check(test_case.exit_code == 3, "stop failure latch exit 3");
  check(test_case.backend.stop_calls == 3,
        "stop failure latch stop attempts");
  check(csv.find("terminal_rejected,stop_failed") != std::string::npos,
        "stop failure latch CSV rejection");
  check(csv.find(",status,") != std::string::npos,
        "stop failure latch status allowed");
  check(countOccurrences(csv, ",task_start,") == 1,
        "stop failure latch only straight task");
  check(csv.find("QUIT_STOP_FAILED") != std::string::npos,
        "stop failure latch end event");
  check(log.find("只允许 status、stop、quit") != std::string::npos,
        "stop failure latch guidance");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " stop_failure_latch\n";
}

void testStopExceptionLatch() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.duration_s = 1.0;
  test_case.backend.stop_throws = true;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(1.5, "goto 1 0");
  test_case.script.add(2.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 10,
        "stop exception latch no extra Move");
  check(test_case.session_state.stop_failed,
        "stop exception latch state remains");
  check(test_case.exit_code == 3, "stop exception latch exit 3");
  check(test_case.backend.stop_calls == 2,
        "stop exception latch stop attempts");
  check(csv.find("fake StopMove exception") != std::string::npos,
        "stop exception latch CSV exception");
  check(csv.find("terminal_rejected,stop_failed") != std::string::npos,
        "stop exception latch CSV rejection");
  check(csv.find("QUIT_STOP_FAILED") != std::string::npos,
        "stop exception latch end event");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " stop_exception_latch\n";
}

void testCtrlCStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.interrupt_at = 0.6;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.stop_calls == 1, "ctrl-c one StopMove");
  check(csv.find("INTERRUPTED") != std::string::npos,
        "ctrl-c CSV INTERRUPTED");
  check(test_case.exit_code == 0, "ctrl-c exit 0");
  check(noCmdAfter(csv, "stop_request"), "ctrl-c no cmd after stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " ctrl_c_stop\n";
}

void testQuitDuringMotion() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(0.6, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 5, "quit during motion 5 Move calls");
  check(test_case.backend.stop_calls == 1, "quit during motion one StopMove");
  check(csv.find("QUIT") != std::string::npos, "quit during motion CSV QUIT");
  check(test_case.exit_code == 0, "quit during motion exit 0");
  check(noCmdAfter(csv, "stop_request"),
        "quit during motion no cmd after stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " quit_during_motion\n";
}

void testStartupTimeout() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.state.received = false;
  test_case.backend.state.finite = false;
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.exit_code == 1, "startup timeout exit 1");
  check(test_case.backend.move_calls == 0, "startup timeout no Move");
  check(test_case.backend.stop_calls == 0, "startup timeout no StopMove");
  check(csv.find("STARTUP_TIMEOUT") != std::string::npos,
        "startup timeout CSV reason");
  std::cout << (failures == before ? "PASS" : "FAIL") << " startup_timeout\n";
}

void testConfigValidation() {
  const int before = failures;
  std::string error;
  g::Config config;
  config.csv_path = "x.csv";

  config.vx_mps = 0.50;
  check(g::validateConfig(config, &error), "accept vx max 0.50");
  config.vx_mps = 0.5001;
  check(!g::validateConfig(config, &error), "reject vx above 0.50");
  config.vx_mps = 0.0;
  check(!g::validateConfig(config, &error), "reject vx below cap");
  config.vx_mps = std::numeric_limits<double>::quiet_NaN();
  check(!g::validateConfig(config, &error), "reject NaN vx");
  config.vx_mps = 0.10;

  config.duration_s = 30.0;
  check(g::validateConfig(config, &error), "accept duration max 30");
  config.duration_s = 30.01;
  check(!g::validateConfig(config, &error), "reject duration above 30");
  config.duration_s = 0.5;
  check(!g::validateConfig(config, &error), "reject short duration");
  config.duration_s = 2.0;

  config.goto_max_speed_mps = 0.30;
  config.goto_approach_speed_mps = 0.30;
  check(g::validateConfig(config, &error), "accept goto 0.30/0.30");
  config.goto_max_speed_mps = 0.50;
  check(g::validateConfig(config, &error), "accept goto max 0.50");
  config.goto_max_speed_mps = 0.5001;
  check(!g::validateConfig(config, &error), "reject goto max above 0.50");
  config.goto_max_speed_mps = 0.30;
  config.goto_approach_speed_mps = 0.31;
  check(!g::validateConfig(config, &error),
        "reject goto approach above max");
  config.goto_approach_speed_mps = 0.0;
  check(!g::validateConfig(config, &error),
        "reject goto approach below min");
  config.goto_approach_speed_mps = 0.30;

  config.vy_mps = 0.1;
  check(!g::validateConfig(config, &error), "reject lateral velocity");
  config.vy_mps = 0.0;
  config.vyaw_radps = 0.1;
  check(!g::validateConfig(config, &error), "reject yaw velocity");
  config.vyaw_radps = 0.0;

  config.csv_path.clear();
  check(!g::validateConfig(config, &error), "reject empty CSV path");
  config.csv_path = "x.csv";

  check(g::validateConfig(config, &error), "accept default-range config");

  config.vx_mps = 0.20;
  config.duration_s = 6.0;
  check(g::validateConfig(config, &error), "accept field 0.20/6 config");

  config.vx_mps = 0.05;
  config.duration_s = 1.0;
  check(g::validateConfig(config, &error), "accept minimum trial config");
  config.vx_mps = 0.50;
  config.duration_s = 30.0;
  check(g::validateConfig(config, &error), "accept maximum trial config");
  std::cout << (failures == before ? "PASS" : "FAIL") << " config\n";
}

}  // namespace

int main() {
  testTransforms();
  testWrapPi();
  testParseCommand();
  testStraightPreserved();
  testGotoForwardAndStop();
  testGotoLeft();
  testGotoApproachFloor();
  testPulseBoundsAndTiming();
  testPlanAndGotoNavigation();
  testPlanTimeoutAndStateStale();
  testVinsControlSource();
  testYawWrap();
  testArrivalStops();
  testTimeoutStops();
  testNewMotionRejected();
  testOriginRules();
  testMoveErrorStops();
  testStateStaleStops();
  testStopFailureExit();
  testStopFailureLatch();
  testStopExceptionLatch();
  testCtrlCStops();
  testQuitDuringMotion();
  testStartupTimeout();
  testConfigValidation();

  if (failures != 0) {
    std::cerr << "selftest failures: " << failures << "\n";
    return 1;
  }
  std::cout << "all selftests passed; no DDS initialized\n";
  return 0;
}
