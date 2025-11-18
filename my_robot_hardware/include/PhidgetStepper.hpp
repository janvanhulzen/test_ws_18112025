#pragma once
#include <phidget22.h>
#include <cstdint>
#include <string>
#include <functional>
#include <chrono>

namespace phg {

// Header-only, exception-free wrapper for Phidget Stepper (STC1005_0, API v111)
// - bool returns (true=ok), lastError_ retains the last error text
// - C-style 0/1 shim via toCStyle()
// - RAII: create in ctor, delete in dtor
// - Event hooks: attach/detach/error/position/velocity/stopped
// NOTE: If your headers have slightly different callback signatures (e.g. position type),
// adjust the trampoline typedefs below (marked).

class Stepper {
public:
  // Public callback types (adapt these if your app prefers different signatures)
  using OnAttach    = std::function<void()>;
  using OnDetach    = std::function<void()>;
  using OnError     = std::function<void(Phidget_ErrorEventCode, const char*)>;
  using OnPosition  = std::function<void(double)>;   // v111 position is treated as double
  using OnVelocity  = std::function<void(double)>;
  using OnStopped   = std::function<void()>;         // v111 stopped event (no args)

  Stepper() : ch_(nullptr) {
    storeError(PhidgetStepper_create(&ch_));
    if (ch_) {
      Phidget_setOnAttachHandler((PhidgetHandle)ch_, &Stepper::c_onAttach, this);
      Phidget_setOnDetachHandler((PhidgetHandle)ch_, &Stepper::c_onDetach, this);
      Phidget_setOnErrorHandler ((PhidgetHandle)ch_, &Stepper::c_onError,  this);

      // v111 change handlers
      PhidgetStepper_setOnPositionChangeHandler(ch_, &Stepper::c_onPosition, this);
      PhidgetStepper_setOnVelocityChangeHandler(ch_, &Stepper::c_onVelocity, this);
      PhidgetStepper_setOnStoppedHandler(ch_, &Stepper::c_onStopped, this);
    }
  }

  Stepper(const Stepper&)            = delete;
  Stepper& operator=(const Stepper&) = delete;

  ~Stepper() {
    close();
    if (ch_) {
      PhidgetStepper_delete(&ch_);
      ch_ = nullptr;
    }
  }

  // ---- Generic Phidget channel config (before open()) ----
  bool setChannel(int channel_number) {
    return storeError(Phidget_setChannel((PhidgetHandle)ch_, channel_number));
  }
  bool setDeviceSerialNumber(int serial) {
    return storeError(Phidget_setDeviceSerialNumber((PhidgetHandle)ch_, serial));
  }
  bool setHubPort(int hubPort) {
    return storeError(Phidget_setHubPort((PhidgetHandle)ch_, hubPort));
  }
  bool setIsHubPortDevice(bool isHubPortDevice) {
    return storeError(Phidget_setIsHubPortDevice((PhidgetHandle)ch_, isHubPortDevice ? 1 : 0));
  }

  // ---- Open/Close ----
  bool open(std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    return storeError(Phidget_openWaitForAttachment((PhidgetHandle)ch_, static_cast<uint32_t>(timeout.count())));
  }
  void close() {
    if (ch_) (void)Phidget_close((PhidgetHandle)ch_);
  }

  // ---- Motion / power ----
  bool setEngaged(bool engaged) {
    return storeError(PhidgetStepper_setEngaged(ch_, engaged ? 1 : 0));
  }
  bool getEngaged(bool& out) const {
    int v{};
    auto rc = PhidgetStepper_getEngaged(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = (v != 0);
    return true;
  }

  // Positioning (v111 uses getPosition + addPositionOffset)
  bool getPosition(double& out) const {
    double v{};
    auto rc = PhidgetStepper_getPosition(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v;
    return true;
  }
  bool addPositionOffset(double offset) {
    return storeError(PhidgetStepper_addPositionOffset(ch_, offset));
  }

  bool getMinPosition(double& out) const {
    double v{};
    auto rc = PhidgetStepper_getMinPosition(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMaxPosition(double& out) const {
    double v{};
    auto rc = PhidgetStepper_getMaxPosition(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  // Targets and velocity
  bool setTargetPosition(double pos) {
    return storeError(PhidgetStepper_setTargetPosition(ch_, pos));
  }
  bool getTargetPosition(double& out) const {
    double v{};
    auto rc = PhidgetStepper_getTargetPosition(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  // Optional async (adjust signature if your header differs)
  // bool setTargetPositionAsync(double pos, std::function<void(PhidgetReturnCode)> done = {}) {
  //   struct Ctx { std::function<void(PhidgetReturnCode)> cb; };
  //   auto ctx = new Ctx{std::move(done)};
  //   auto rc = PhidgetStepper_setTargetPosition_async(
  //       ch_, pos,
  //       [](PhidgetStepperHandle /*h*/, void* u, PhidgetReturnCode result) {
  //         std::unique_ptr<Ctx> hold(static_cast<Ctx*>(u));
  //         if (hold->cb) hold->cb(result);
  //       }, ctx);
  //   return storeError(rc);
  // }

  bool getVelocity(double& out) const {
    double v{};
    auto rc = PhidgetStepper_getVelocity(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  bool setVelocityLimit(double v) { return storeError(PhidgetStepper_setVelocityLimit(ch_, v)); }
  bool getVelocityLimit(double& out) const {
    double v{}; auto rc = PhidgetStepper_getVelocityLimit(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMinVelocityLimit(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMinVelocityLimit(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMaxVelocityLimit(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMaxVelocityLimit(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  bool setAcceleration(double a) { return storeError(PhidgetStepper_setAcceleration(ch_, a)); }
  bool getAcceleration(double& out) const {
    double v{}; auto rc = PhidgetStepper_getAcceleration(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMinAcceleration(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMinAcceleration(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMaxAcceleration(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMaxAcceleration(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  bool getIsMoving(bool& out) const {
    int v{}; auto rc = PhidgetStepper_getIsMoving(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = (v != 0); return true;
  }

  // Power & current
  bool setCurrentLimit(double amps) { return storeError(PhidgetStepper_setCurrentLimit(ch_, amps)); }
  bool getCurrentLimit(double& out) const {
    double v{}; auto rc = PhidgetStepper_getCurrentLimit(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMinCurrentLimit(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMinCurrentLimit(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMaxCurrentLimit(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMaxCurrentLimit(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  bool setHoldingCurrentLimit(double amps) { return storeError(PhidgetStepper_setHoldingCurrentLimit(ch_, amps)); }
  bool getHoldingCurrentLimit(double& out) const {
    double v{}; auto rc = PhidgetStepper_getHoldingCurrentLimit(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  // Control mode
  bool setControlMode(PhidgetStepper_ControlMode m) {
    return storeError(PhidgetStepper_setControlMode(ch_, m));
  }
  bool getControlMode(PhidgetStepper_ControlMode& out) const {
    PhidgetStepper_ControlMode v{};
    auto rc = PhidgetStepper_getControlMode(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  // Data interval / rate
  bool setDataInterval(uint32_t ms) { return storeError(PhidgetStepper_setDataInterval(ch_, ms)); }
  bool getDataInterval(uint32_t& out) const {
    uint32_t v{}; auto rc = PhidgetStepper_getDataInterval(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMinDataInterval(uint32_t& out) const {
    uint32_t v{}; auto rc = PhidgetStepper_getMinDataInterval(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMaxDataInterval(uint32_t& out) const {
    uint32_t v{}; auto rc = PhidgetStepper_getMaxDataInterval(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  bool setDataRate(double hz) { return storeError(PhidgetStepper_setDataRate(ch_, hz)); }
  bool getDataRate(double& out) const {
    double v{}; auto rc = PhidgetStepper_getDataRate(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMinDataRate(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMinDataRate(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMaxDataRate(double& out) const {
    double v{}; auto rc = PhidgetStepper_getMaxDataRate(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  // Failsafe
  bool enableFailsafe(uint32_t timeout_ms) { return storeError(PhidgetStepper_enableFailsafe(ch_, timeout_ms)); }
  bool resetFailsafe()                     { return storeError(PhidgetStepper_resetFailsafe(ch_)); }
  bool getMinFailsafeTime(uint32_t& out) const {
    uint32_t v{}; auto rc = PhidgetStepper_getMinFailsafeTime(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }
  bool getMaxFailsafeTime(uint32_t& out) const {
    uint32_t v{}; auto rc = PhidgetStepper_getMaxFailsafeTime(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  // Rescale factor
  bool setRescaleFactor(double f) { return storeError(PhidgetStepper_setRescaleFactor(ch_, f)); }
  bool getRescaleFactor(double& out) const {
    double v{}; auto rc = PhidgetStepper_getRescaleFactor(ch_, &v);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = v; return true;
  }

  // Device info
  bool getDeviceName(std::string& out) const {
    const char* name = nullptr;
    auto rc = Phidget_getDeviceName((PhidgetHandle)ch_, &name);
    if (!const_cast<Stepper*>(this)->storeError(rc)) return false;
    out = name ? name : "";
    return true;
  }
  const std::string& lastError() const { return lastError_; }
  bool ok() const { return lastError_.empty(); }

  // Event hooks
  void onAttach   (OnAttach f)   { onAttach_    = std::move(f); }
  void onDetach   (OnDetach f)   { onDetach_    = std::move(f); }
  void onError    (OnError f)    { onError_     = std::move(f); }
  void onPosition (OnPosition f) { onPosition_  = std::move(f); }
  void onVelocity (OnVelocity f) { onVelocity_  = std::move(f); }
  void onStopped  (OnStopped f)  { onStopped_   = std::move(f); }

  // Your 0/1 convention
  static int toCStyle(bool success) { return success ? 0 : 1; }

private:
  PhidgetStepperHandle ch_;
  std::string lastError_;

  bool storeError(PhidgetReturnCode rc) const {
    auto self = const_cast<Stepper*>(this);
    if (rc == EPHIDGET_OK) {
      self->lastError_.clear();
      return true;
    }
    const char* desc = nullptr;
    Phidget_getErrorDescription(rc, &desc);
    self->lastError_ = desc ? desc : "Unknown Phidget error";
    return false;
  }

  // ---- C callback trampolines (adjust signatures if needed) ----
  static void CCONV c_onAttach(PhidgetHandle, void* ctx) {
    auto* self = static_cast<Stepper*>(ctx);
    if (self && self->onAttach_) self->onAttach_();
  }
  static void CCONV c_onDetach(PhidgetHandle, void* ctx) {
    auto* self = static_cast<Stepper*>(ctx);
    if (self && self->onDetach_) self->onDetach_();
  }
  static void CCONV c_onError(PhidgetHandle, void* ctx, Phidget_ErrorEventCode code, const char* desc) {
    auto* self = static_cast<Stepper*>(ctx);
    if (self && self->onError_) self->onError_(code, desc ? desc : "");
  }
  // v111: position/velocity likely double
  static void CCONV c_onPosition(PhidgetStepperHandle, void* ctx, double position) {
    auto* self = static_cast<Stepper*>(ctx);
    if (self && self->onPosition_) self->onPosition_(position);
  }
  static void CCONV c_onVelocity(PhidgetStepperHandle, void* ctx, double velocity) {
    auto* self = static_cast<Stepper*>(ctx);
    if (self && self->onVelocity_) self->onVelocity_(velocity);
  }
  static void CCONV c_onStopped(PhidgetStepperHandle, void* ctx) {
    auto* self = static_cast<Stepper*>(ctx);
    if (self && self->onStopped_) self->onStopped_();
  }

  // Stored handlers
  OnAttach  onAttach_{};
  OnDetach  onDetach_{};
  OnError   onError_{};
  OnPosition onPosition_{};
  OnVelocity onVelocity_{};
  OnStopped  onStopped_{};
};

} // namespace phg
