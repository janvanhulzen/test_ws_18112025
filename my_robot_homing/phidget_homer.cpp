// phidget_homer.cpp
#include <phidget22.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

static std::atomic<bool> g_stop{false};
void sigint_handler(int) { g_stop = true; }

static inline void pthrow(PhidgetReturnCode rc, const char* what) {
  if (rc != EPHIDGET_OK) {
    const char* err = nullptr; Phidget_getErrorDescription(rc, &err);
    std::ostringstream oss;
    oss << what << " : " << (err ? err : "Phidget error");
    throw std::runtime_error(oss.str());
  }
}

// ---- simple CLI ----
struct Args {
  int stepper_serial = 0;               // VINT hub serial for steppers
  std::vector<int> stepper_ports;       // VINT ports for steppers (one per axis)
  std::vector<int> stepper_channels;    // optional, defaults to 0 for each
  int daq_serial = 0;                   // VINT hub serial for DAQ1301
  int daq_port = 0;                     // hub port where DAQ1301 sits
  std::vector<int> di_min;              // DI channels (min switch) per axis
  std::vector<int> di_max;              // DI channels (max switch) per axis
  bool switches_nc = false;             // normally-closed wiring?
  // motion
  double accel = 2000.0;
  double vel = 200.0;
  double backoff = 200.0;
  double timeout_s = 20.0;
  double mid_tol = 1.0;                 // position tolerance at midpoint
  // runner
  bool dry_run = false;
};

static std::vector<int> parse_list(const std::string& s) {
  std::vector<int> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (tok.empty()) continue;
    out.push_back(std::stoi(tok));
  }
  return out;
}

static void usage(const char* prog) {
  std::cout <<
R"(Phidget multi-axis homing (standalone)

Required:
  --stepper-serial N             VINT hub serial with steppers
  --stepper-ports p1,p2,...      VINT hub ports per axis (e.g. 1,2,3,4,5)
  --daq-serial N                 VINT hub serial with DAQ1301 switches
  --daq-port N                   VINT hub port where DAQ1301 is plugged
  --di-min m1,m2,...             DI channels (min) per axis (e.g. 0,1,2,3,4)
  --di-max M1,M2,...             DI channels (max) per axis (e.g. 11,12,13,14,15)

Optional:
  --stepper-channels c1,c2,...   Phidget stepper channel per axis (default: 0)
  --nc                           switches are Normally-Closed (logical trip = raw==0)
  --accel A                      homing acceleration (default 2000)
  --vel V                        homing velocity limit (default 200)
  --backoff B                    backoff distance after tripping (default 200)
  --timeout T                    per-direction timeout in seconds (default 20)
  --midtol E                     midpoint position tolerance (default 1.0)
  --dry-run                      don’t move, just print what would happen

Example:
  sudo ./phidget_homer \
    --stepper-serial 740266 --stepper-ports 1,2,3,4,5 \
    --daq-serial 528369 --daq-port 0 \
    --di-min 0,1,2,3,4 --di-max 11,12,13,14,15 --accel 1500 --vel 150 --backoff 150

Note: Close any process that already opened these Phidget devices.
)" << std::endl;
  (void)prog;
}

static Args parse_args(int argc, char** argv) {
  Args a;
  for (int i=1; i<argc; ++i) {
    std::string s = argv[i];
    auto req = [&](const char* opt)->std::string{
      if (i+1 >= argc) throw std::runtime_error(std::string("missing value for ")+opt);
      return argv[++i];
    };
    if (s=="--stepper-serial") a.stepper_serial = std::stoi(req(s.c_str()));
    else if (s=="--stepper-ports") a.stepper_ports = parse_list(req(s.c_str()));
    else if (s=="--stepper-channels") a.stepper_channels = parse_list(req(s.c_str()));
    else if (s=="--daq-serial") a.daq_serial = std::stoi(req(s.c_str()));
    else if (s=="--daq-port") a.daq_port = std::stoi(req(s.c_str()));
    else if (s=="--di-min") a.di_min = parse_list(req(s.c_str()));
    else if (s=="--di-max") a.di_max = parse_list(req(s.c_str()));
    else if (s=="--nc") a.switches_nc = true;
    else if (s=="--accel") a.accel = std::stod(req(s.c_str()));
    else if (s=="--vel") a.vel = std::stod(req(s.c_str()));
    else if (s=="--backoff") a.backoff = std::stod(req(s.c_str()));
    else if (s=="--timeout") a.timeout_s = std::stod(req(s.c_str()));
    else if (s=="--midtol") a.mid_tol = std::stod(req(s.c_str()));
    else if (s=="--dry-run") a.dry_run = true;
    else if (s=="-h" || s=="--help") { usage(argv[0]); std::exit(0); }
    else {
      std::ostringstream oss; oss << "Unknown option: " << s;
      throw std::runtime_error(oss.str());
    }
  }
  if (a.stepper_serial==0 || a.daq_serial==0 ||
      a.stepper_ports.empty() || a.di_min.empty() || a.di_max.empty()) {
    throw std::runtime_error("missing required arguments; use --help");
  }
  if (a.di_min.size()!=a.stepper_ports.size() || a.di_max.size()!=a.stepper_ports.size()) {
    throw std::runtime_error("di_min/di_max size must match stepper_ports size");
  }
  if (a.stepper_channels.empty()) a.stepper_channels = std::vector<int>(a.stepper_ports.size(), 0);
  if (a.stepper_channels.size()!=a.stepper_ports.size()) {
    throw std::runtime_error("stepper_channels size must match stepper_ports size");
  }
  return a;
}

// ---- RAII wrappers ----
struct Stepper {
  PhidgetStepperHandle h=nullptr;
  ~Stepper(){ if(h){ Phidget_close((PhidgetHandle)h); PhidgetStepper_delete(&h);} }
};
struct DI {
  PhidgetDigitalInputHandle h=nullptr;
  ~DI(){ if(h){ Phidget_close((PhidgetHandle)h); PhidgetDigitalInput_delete(&h);} }
};

static bool di_logical(int raw, bool nc){ return nc ? (raw==0) : (raw!=0); }

static Stepper open_stepper(int serial, int port, int channel, double accel, double vel) {
  Stepper s;
  pthrow(PhidgetStepper_create(&s.h), "create stepper");
  pthrow(Phidget_setDeviceSerialNumber((PhidgetHandle)s.h, serial), "set serial");
  pthrow(Phidget_setHubPort((PhidgetHandle)s.h, port), "set hub port");
  pthrow(Phidget_setIsHubPortDevice((PhidgetHandle)s.h, 0), "is hub port device");
  pthrow(Phidget_setChannel((PhidgetHandle)s.h, channel), "set stepper channel");

  auto rc = Phidget_openWaitForAttachment((PhidgetHandle)s.h, 5000);
  if (rc != EPHIDGET_OK) pthrow(rc, "attach stepper");

  pthrow(PhidgetStepper_setAcceleration(s.h, accel), "set accel");
  pthrow(PhidgetStepper_setVelocityLimit(s.h, vel), "set vel limit");
  pthrow(PhidgetStepper_setEngaged(s.h, 1), "engage");
  return s;
}

static DI open_di(int serial, int hub_port, int channel) {
  DI d;
  pthrow(PhidgetDigitalInput_create(&d.h), "create di");
  pthrow(Phidget_setDeviceSerialNumber((PhidgetHandle)d.h, serial), "di set serial");
  pthrow(Phidget_setHubPort((PhidgetHandle)d.h, hub_port), "di set hub port");
  pthrow(Phidget_setIsHubPortDevice((PhidgetHandle)d.h, 0), "di is hub port device");
  pthrow(Phidget_setChannel((PhidgetHandle)d.h, channel), "di set channel");
  auto rc = Phidget_openWaitForAttachment((PhidgetHandle)d.h, 5000);
  if (rc != EPHIDGET_OK) pthrow(rc, "attach di");
  return d;
}

static bool wait_for_di(DI& d, bool want_tripped, bool nc, double timeout_s) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds((int)(timeout_s*1000));
  while (!g_stop && std::chrono::steady_clock::now() < deadline) {
    int raw=0; PhidgetDigitalInput_getState(d.h, &raw);
    if (di_logical(raw, nc) == want_tripped) return true;
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

static double current_pos(Stepper& s) {
  double p=0.0; PhidgetStepper_getPosition(s.h, &p); return p;
}

static void run_velocity(Stepper& s, double v) {
  pthrow(PhidgetStepper_setControlMode(s.h, CONTROL_MODE_RUN), "set run mode");
  // Velocity limit magnitude already set; just set signed target velocity.
  pthrow(PhidgetStepper_setVelocityLimit(s.h, v), "set target vel");
}

static void stop_velocity(Stepper& s) {
  pthrow(PhidgetStepper_setVelocityLimit(s.h, 0.0), "stop velocity");
  pthrow(PhidgetStepper_setControlMode(s.h, CONTROL_MODE_STEP), "set pos mode");
}

static void move_to(Stepper& s, double target, double tol, double timeout_s) {
  pthrow(PhidgetStepper_setControlMode(s.h, CONTROL_MODE_STEP), "pos mode");
  pthrow(PhidgetStepper_setTargetPosition(s.h, target), "set target pos");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds((int)(timeout_s*1000));
  while (!g_stop && std::chrono::steady_clock::now() < deadline) {
    double cur = current_pos(s);
    if (std::abs(cur - target) <= tol) return;
    std::this_thread::sleep_for(10ms);
  }
  std::cerr << "[WARN] move_to timeout\n";
}

static void backoff_from_switch(Stepper& s, double delta, double tol, double timeout_s) {
  double cur = current_pos(s);
  move_to(s, cur + delta, tol, timeout_s);
}

static void home_axis(
  Stepper& stp, DI& di_min, DI& di_max, const Args& a, size_t axis_idx)
{
  std::cout << "Axis#" << axis_idx
            << " : homing (vel=" << a.vel << ", accel=" << a.accel
            << ", backoff=" << a.backoff << ")\n";

  if (a.dry_run) {
    std::cout << "  [dry-run] skipping motion\n";
    return;
  }

  // 1) Jog negative until MIN trips
  run_velocity(stp, -std::abs(a.vel));
  if (!wait_for_di(di_min, /*want_tripped=*/true, a.switches_nc, a.timeout_s)) {
    stop_velocity(stp);
    throw std::runtime_error("timeout waiting for MIN trip");
  }
  stop_velocity(stp);
  double pos_min = current_pos(stp);
  std::cout << "  MIN at " << pos_min << "\n";

  // 2) Back off MIN to clear
  backoff_from_switch(stp, +a.backoff, a.mid_tol, a.timeout_s);
  if (!wait_for_di(di_min, /*want_tripped=*/false, a.switches_nc, a.timeout_s))
    throw std::runtime_error("timeout clearing MIN");

  // 3) Jog positive until MAX trips
  run_velocity(stp, +std::abs(a.vel));
  if (!wait_for_di(di_max, /*want_tripped=*/true, a.switches_nc, a.timeout_s)) {
    stop_velocity(stp);
    throw std::runtime_error("timeout waiting for MAX trip");
  }
  stop_velocity(stp);
  double pos_max = current_pos(stp);
  std::cout << "  MAX at " << pos_max << "\n";

  // 4) Back off MAX to clear
  backoff_from_switch(stp, -a.backoff, a.mid_tol, a.timeout_s);
  if (!wait_for_di(di_max, /*want_tripped=*/false, a.switches_nc, a.timeout_s))
    throw std::runtime_error("timeout clearing MAX");

  // 5) Midpoint & zero
  const double mid = 0.5*(pos_min + pos_max);
  std::cout << "  MID = " << mid << " (span " << (pos_max - pos_min) << ")\n";
  move_to(stp, mid, a.mid_tol, a.timeout_s);
  double nowp = current_pos(stp);
  pthrow(PhidgetStepper_addPositionOffset(stp.h, -nowp), "zero offset");
  std::cout << "  Zeroed. Current pos = 0\n";
}

int main(int argc, char** argv) {
  std::signal(SIGINT, sigint_handler);
  try {
    Args a = parse_args(argc, argv);

    std::cout << "=== Phidget Homer ===\n";
    std::cout << "Axes: " << a.stepper_ports.size() << "\n";

    // Open devices
    std::vector<Stepper> steppers(a.stepper_ports.size());
    std::vector<DI> di_min(a.stepper_ports.size());
    std::vector<DI> di_max(a.stepper_ports.size());

    for (size_t i=0; i<steppers.size(); ++i) {
      if (g_stop) throw std::runtime_error("interrupted");
      std::cout << "Opening stepper port " << a.stepper_ports[i]
                << " ch " << a.stepper_channels[i] << " ...\n";
      steppers[i] = open_stepper(a.stepper_serial, a.stepper_ports[i], a.stepper_channels[i],
                                 a.accel, a.vel);
    }
    for (size_t i=0; i<steppers.size(); ++i) {
      std::cout << "Opening DI min ch " << a.di_min[i] << " / max ch " << a.di_max[i] << " ...\n";
      di_min[i] = open_di(a.daq_serial, a.daq_port, a.di_min[i]);
      di_max[i] = open_di(a.daq_serial, a.daq_port, a.di_max[i]);
    }

    // Home sequentially (safer mechanically). Parallel is possible if you want.
    for (size_t i=0; i<steppers.size(); ++i) {
      if (g_stop) throw std::runtime_error("interrupted");
      home_axis(steppers[i], di_min[i], di_max[i], a, i);
    }

    // Disengage (optional)
    for (auto& s : steppers) {
      if (s.h) PhidgetStepper_setEngaged(s.h, 0);
    }

    std::cout << "Homing complete. All devices closed.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] " << e.what() << "\n";
    return 2;
  }
}
