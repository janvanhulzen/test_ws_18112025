#pragma once
#include <string>
#include <chrono>
#include <optional>
#include "PhidgetStepper.hpp"   // your header-only Phidgets C++ wrapper

namespace armhw {

struct StepperParams {
  // Identification (pre-open)
  int serial = 0;
  int hub_port = 0;
  int channel = 0;
  bool is_hub_port_device = false;

  // Open timeout
  int open_timeout_ms = 5000;

  // Units & control
  double rescale_factor = 1.0;                 // user units per step
  PhidgetStepper_ControlMode control_mode = CONTROL_MODE_STEP;

  // Motion/current limits
  double acceleration = 10000.0;
  double velocity_limit = 2000.0;
  double current_limit = 1.2;
  double holding_current_limit = 0.3;

  // Optional startup actions
  bool zero_on_configure = false;              // software rebase to 0 after open
};

class StepperDriver {
public:
  StepperDriver() = default;

  // --- Lifecycle-like API ---
  // CONFIGURE: open device + apply all settings, but DO NOT engage.
  // Returns true on success; on error, fills last_error().
  bool configure(const StepperParams& p) {
    params_ = p;
    stepper_ = std::make_unique<phg::Stepper>();
    if (!stepper_) { last_error_ = "No memory for stepper"; return false; }

    // Identify channel before open
    RET( stepper_->setDeviceSerialNumber(p.serial),            "setDeviceSerialNumber" );
    RET( stepper_->setHubPort(p.hub_port),                     "setHubPort" );
    RET( stepper_->setIsHubPortDevice(p.is_hub_port_device),   "setIsHubPortDevice" );
    RET( stepper_->setChannel(p.channel),                      "setChannel" );

    // Open (attachment)
    RET( stepper_->open(std::chrono::milliseconds(p.open_timeout_ms)), "openWaitForAttachment" );

    // Set Rescale Factor again to ensure correct scaling after setting limits
    RET( stepper_->setRescaleFactor(p.rescale_factor),       "setRescaleFactor" );

    // Mode & scaling
    //RET( stepper_->setControlMode(p.control_mode),   "setControlMode" );
    RET(stepper_->setControlMode(CONTROL_MODE_STEP),  "setControlMode");
    RET( stepper_->setRescaleFactor(p.rescale_factor), "setRescaleFactor" );

    // Limits
    // now limits in rad/s, rad/s^2 (because rescale is active)
    RET( stepper_->setAcceleration(p.acceleration),          "setAcceleration" );
    RET( stepper_->setVelocityLimit(p.velocity_limit),       "setVelocityLimit" );
    RET( stepper_->setCurrentLimit(p.current_limit),         "setCurrentLimit" );
    RET( stepper_->setHoldingCurrentLimit(p.holding_current_limit), "setHoldingCurrentLimit" );



    // Ensure disengaged in configure phase
    (void)stepper_->setEngaged(false);

    // Optional zeroing to make "now" be 0
    if (p.zero_on_configure) {
      double pos=0; RET( stepper_->getPosition(pos), "getPosition" );
      RET( stepper_->addPositionOffset(-pos), "addPositionOffset(-pos)" );
    }

    configured_ = true;
    active_ = false;
    cmd_target_.reset();
    last_error_.clear();
    return true;
  }

  // ACTIVATE: engage the driver (safe to start motion)
  bool activate() {
    if (!configured()) { last_error_ = "activate() before configure()"; return false; }
    RET( stepper_->setEngaged(true), "setEngaged(true)" );
    active_ = true;
    return true;
  }

  // DEACTIVATE: disengage (stop holding torque)
  bool deactivate() {
    if (!configured()) return true;
    (void)stepper_->setEngaged(false);
    active_ = false;
    return true;
  }

  // CLEANUP: close and release
  bool cleanup() {
    if (stepper_) {
      (void)stepper_->setEngaged(false);
      stepper_->close();
      stepper_.reset();
    }
    configured_ = false;
    active_ = false;
    cmd_target_.reset();
    return true;
  }

  // --- Realtime-ish hooks for ros2_control integration ---
  // WRITE: accept the latest commanded target position (absolute, in rescaled units).
  // Store only; actual device write happens here immediately (synchronous) to keep it simple.
  bool write(double target_abs) {
    if (!active_) return true;  // ignore commands when not active
    cmd_target_ = target_abs;
    return stepper_->setTargetPosition(target_abs);
  }

  // READ: fetch current state for state interfaces
  bool read(double& position_out, double& velocity_out, bool& moving_out) {
    if (!configured()) { last_error_ = "read() before configure()"; return false; }
    double p=0, v=0; bool m=false;
    RET( stepper_->getPosition(p), "getPosition" );
    RET( stepper_->getVelocity(v), "getVelocity" );
    RET( stepper_->getIsMoving(m), "getIsMoving" );
    position_out = p; velocity_out = v; moving_out = m;
    return true;
  }

  // Utilities you might call from the hardware interface
  bool zeroPosition() {
    double p=0; RET( stepper_->getPosition(p), "getPosition" );
    return stepper_->addPositionOffset(-p);
  }
  bool moveBy(double delta) {
    double p=0; RET( stepper_->getPosition(p), "getPosition" );
    return write(p + delta);
  }
  bool waitUntilStopped(uint32_t timeout_ms, uint32_t poll_ms = 20) {
    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
      bool moving=false; RET( stepper_->getIsMoving(moving), "getIsMoving" );
      if (!moving) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();
      if (dt > static_cast<long long>(timeout_ms)) { last_error_ = "waitUntilStopped timeout"; return false; }
    }
  }

  // Accessors
  bool configured() const { return configured_ && stepper_ != nullptr; }
  bool active() const { return active_; }
  const StepperParams& params() const { return params_; }
  const std::string& last_error() const { return last_error_; }
  phg::Stepper* raw() const { return stepper_.get(); } // if you need direct access

private:
  // Helper macro-ish: capture error string and return false
  bool RET(bool ok, const char* what) {
    if (ok) return true;
    last_error_ = stepper_ ? (std::string(what) + ": " + stepper_->lastError())
                           : (std::string(what) + ": no stepper");
    return false;
  }

  std::unique_ptr<phg::Stepper> stepper_;
  StepperParams params_{};
  std::optional<double> cmd_target_{};
  bool configured_ = false;
  bool active_ = false;
  std::string last_error_;
};

} // namespace armhw
