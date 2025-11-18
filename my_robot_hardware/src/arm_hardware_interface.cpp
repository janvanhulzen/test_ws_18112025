
#include "arm_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <unordered_map>
#include <string>
#include <thread>  

// STATE: position + velocity per joint
std::vector<hardware_interface::StateInterface>
arm_hardware::ArmHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> v;
  v.reserve(state_position_.size() * 2);
  for (std::size_t i = 0; i < state_position_.size(); ++i) {
    const auto &name = info_.joints[i].name;  // e.g., "axis_1"
    v.emplace_back(name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
    v.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
  }

  // debug addresses (optional)
  // RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
  //             "export_state ptrs pos[0]=%p vel[0]=%p",
  //             (void*)&state_position_[0], (void*)&state_velocity_[0]);
  return v;
}

// COMMAND: position per joint (what JTC writes)
std::vector<hardware_interface::CommandInterface>
arm_hardware::ArmHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> v;
  v.reserve(cmd_position_.size());
  for (std::size_t i = 0; i < cmd_position_.size(); ++i) {
    const auto &name = info_.joints[i].name;
    v.emplace_back(name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
  }

  // debug addresses (optional)
  // RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
  //             "export_cmd ptrs: %p %p %p %p %p",
  //             (void*)&cmd_position_[0],
  //             cmd_position_.size() > 1 ? (void*)&cmd_position_[1] : nullptr,
  //             cmd_position_.size() > 2 ? (void*)&cmd_position_[2] : nullptr,
  //             cmd_position_.size() > 3 ? (void*)&cmd_position_[3] : nullptr,
  //             cmd_position_.size() > 4 ? (void*)&cmd_position_[4] : nullptr);
  return v;
}


using hardware_interface::CallbackReturn;

namespace {

// helpers to read flat string params from HardwareInfo::hardware_parameters
template <typename T>
bool get_param(const std::unordered_map<std::string, std::string>& map,
               const std::string& key, T& out);

template <>
bool get_param<int>(const std::unordered_map<std::string, std::string>& map,
                    const std::string& key, int& out) {
  auto it = map.find(key);
  if (it == map.end()) return false;
  try { out = std::stoi(it->second); } catch (...) { return false; }
  return true;
}

template <>
bool get_param<double>(const std::unordered_map<std::string, std::string>& map,
                       const std::string& key, double& out) {
  auto it = map.find(key);
  if (it == map.end()) return false;
  try { out = std::stod(it->second); } catch (...) { return false; }
  return true;
}

template <>
bool get_param<bool>(const std::unordered_map<std::string, std::string>& map,
                     const std::string& key, bool& out) {
  auto it = map.find(key);
  if (it == map.end()) return false;
  const auto& s = it->second;
  if (s == "true" || s == "1")  { out = true;  return true; }
  if (s == "false"|| s == "0")  { out = false; return true; }
  return false;
}

inline std::string key(const std::string& joint_name, const char* leaf) {
  // e.g., "axis_1.serial"
  return joint_name + "." + leaf;
}

} // anonymous namespace

namespace arm_hardware
{

    // -----------------------------------------------------------------------------
    // on_init(): called once when the plugin is created by ros2_control.
    // - Read and cache metadata from URDF / hardware_info (via base class).
    // - Parse parameters you care about (hub, port, channel etc).
    // - DO NOT touch real hardware here — just prepare configuration.
    // - Build the shared 'System' bundle (but do not open or activate the hub yet).
    // -----------------------------------------------------------------------------


CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  // IMPORTANT: call base to let ros2_control do its parsing/validation
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                 "Base SystemInterface::on_init failed.");
    return CallbackReturn::ERROR;
  }

  if (info.joints.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                 "No joints defined in <ros2_control>.");
    return CallbackReturn::ERROR;
  }

  // Size containers for all joints
  params_.resize(info.joints.size());
  drivers_.resize(info.joints.size()); // created in on_configure()
  cmd_position_.assign(info.joints.size(), 0.0);
  state_position_.assign(info.joints.size(), 0.0);
  state_velocity_.assign(info.joints.size(), 0.0);
  state_moving_.assign(info.joints.size(), 0.0);

  const auto& pmap = info.hardware_parameters;

  // Parse per-joint parameters
  for (std::size_t i = 0; i < info.joints.size(); ++i) {
    const auto& j = info.joints[i];  // has .name = "axis_1", etc.

    // Required
    bool ok = true;
    ok &= get_param(pmap, key(j.name, "serial"),   params_[i].serial);
    ok &= get_param(pmap, key(j.name, "hub_port"), params_[i].hub_port);
    ok &= get_param(pmap, key(j.name, "channel"),  params_[i].channel);

    // Optional (defaults are already set in StepperParams)
    (void) get_param(pmap, key(j.name, "is_hub_port_device"),    params_[i].is_hub_port_device);
    (void) get_param(pmap, key(j.name, "open_timeout_ms"),       params_[i].open_timeout_ms);
    (void) get_param(pmap, key(j.name, "rescale_factor"),        params_[i].rescale_factor);
    (void) get_param(pmap, key(j.name, "acceleration"),          params_[i].acceleration);
    (void) get_param(pmap, key(j.name, "velocity_limit"),        params_[i].velocity_limit);
    (void) get_param(pmap, key(j.name, "current_limit"),         params_[i].current_limit);
    (void) get_param(pmap, key(j.name, "holding_current_limit"), params_[i].holding_current_limit);
    (void) get_param(pmap, key(j.name, "zero_on_configure"),     params_[i].zero_on_configure);

    params_[i].control_mode = CONTROL_MODE_STEP; // fixed for now

    if (!ok) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                   "Missing required params for joint '%s' (need %s.serial, %s.hub_port, %s.channel).",
                   j.name.c_str(), j.name.c_str(), j.name.c_str(), j.name.c_str());
      return CallbackReturn::ERROR;
    }

    cmd_position_[i] = 0.0; // initial command
  }

  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
              "on_init: parsed %zu joints from URDF.", info.joints.size());

  configured_ = false;
  active_ = false;
  return CallbackReturn::SUCCESS;
}


// -----------------------------------------------------------------------------
// on_configure(): UNCONFIGURED -> INACTIVE
// Touch real hardware for the first time:
//  - create a StepperDriver per joint
//  - call configure(params_[i])
//  - DO NOT engage here (that happens in on_activate())
// -----------------------------------------------------------------------------
hardware_interface::CallbackReturn
arm_hardware::ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (params_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                 "on_configure(): params_ is empty. Did on_init() run?");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Ensure driver slots exist
  drivers_.resize(params_.size());

  // Create + configure each driver (no engage)
  for (std::size_t i = 0; i < params_.size(); ++i) {
    drivers_[i] = std::make_unique<armhw::StepperDriver>();
    if (!drivers_[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                   "on_configure(): failed to allocate driver for joint %zu", i);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!drivers_[i]->configure(params_[i])) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                   "on_configure(): configure() failed on joint %zu: %s",
                   i, drivers_[i]->last_error().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  configured_ = true;
  active_ = false;

  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
              "on_configure(): configured %zu Phidget steppers (not engaged).",
              drivers_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// end on_configure()

// -----------------------------------------------------------------------------
// on_activate(): INACTIVE -> ACTIVE
// - Engage each motor
// - Initialize state buffers by reading current position
// -----------------------------------------------------------------------------

hardware_interface::CallbackReturn
arm_hardware::ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                 "on_activate() called before on_configure().");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (drivers_.empty() || drivers_.size() != params_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                 "on_activate(): drivers_ not initialized.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Engage each motor and prime buffers
  for (std::size_t i = 0; i < drivers_.size(); ++i) {
    auto & drv = drivers_[i];
    if (!drv) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                   "on_activate(): driver[%zu] is null.", i);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!drv->activate()) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                   "on_activate(): activate() failed on joint %zu: %s",
                   i, drv->last_error().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read initial state (best effort)
    double p = 0.0, v = 0.0; bool m = false;
    if (!drv->read(p, v, m)) {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                  "on_activate(): initial read failed on joint %zu: %s",
                  i, drv->last_error().c_str());
      // keep zeros if read fails
      p = 0.0; v = 0.0; m = false;
    }

    state_position_[i] = p;
    state_velocity_[i] = v;
    state_moving_[i]   = m ? 1.0 : 0.0;

    // Initialize command to current position to avoid jumps
    cmd_position_[i] = p;
  }

  active_ = true;
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
              "on_activate(): engaged %zu steppers.", drivers_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}



// -----------------------------------------------------------------------------
// on_deactivate(): ACTIVE -> INACTIVE
// - Disengage each motor
// - Optionally nudge commands to current positions to avoid jumps on next activate
// -----------------------------------------------------------------------------

hardware_interface::CallbackReturn
arm_hardware::ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // If we were never configured, nothing to do.
  if (drivers_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
                "on_deactivate(): no drivers to deactivate.");
    active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Optional: nudge commands to current positions to avoid a final jump
  for (std::size_t i = 0; i < drivers_.size(); ++i) {
    if (!drivers_[i]) continue;
    double p = 0.0, v = 0.0; bool m = false;
    if (drivers_[i]->read(p, v, m)) {
      cmd_position_[i]   = p;
      state_position_[i] = p;
      state_velocity_[i] = v;
      state_moving_[i]   = m ? 1.0 : 0.0;
    }
  }

  // Small settle delay (optional)
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Disengage each motor
  for (std::size_t i = 0; i < drivers_.size(); ++i) {
    if (!drivers_[i]) continue;
    if (!drivers_[i]->deactivate()) {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                  "on_deactivate(): deactivate() failed on joint %zu: %s",
                  i, drivers_[i]->last_error().c_str());
      // keep going; best-effort shutdown
    }
  }

  active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
              "on_deactivate(): all steppers disengaged.");
  return hardware_interface::CallbackReturn::SUCCESS;
}
// end on_deactivate()

// -----------------------------------------------------------------------------
// read(): read current position/velocity from each driver
// - called periodically by ros2_control
// -----------------------------------------------------------------------------

 hardware_interface::return_type
arm_hardware::ArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (drivers_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                "read(): no drivers present (not configured?)");
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  (void)dt; // not needed if we trust hardware position; kept if you later want integration

  // Small velocity deadband (units = your rescaled units / s)
  constexpr double V_EPS = 0.03;

  for (std::size_t i = 0; i < drivers_.size(); ++i) {
    if (!drivers_[i]) {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                  "read(): driver[%zu] is null", i);
      return hardware_interface::return_type::ERROR;
    }

    double p = 0.0, v = 0.0; bool m = false;
    if (!drivers_[i]->read(p, v, m)) {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                  "read(): driver[%zu]->read failed: %s", i, drivers_[i]->last_error().c_str());
      return hardware_interface::return_type::ERROR;
    }
  
    // Sanity checks for nan/inf;
    if (!std::isfinite(p)) p = 0.0;
    if (!std::isfinite(v)) v = 0.0;

    // Apply deadband to velocity to reduce chatter
    if (std::abs(v) < V_EPS) v = 0.0;

    state_position_[i] = p;          // hardware-reported absolute position (already rescaled)
    state_velocity_[i] = v;          // hardware-reported velocity (rescaled units/s)
    state_moving_[i]   = m ? 1.0 : 0.0;
  }

  return hardware_interface::return_type::OK;
}

// -----------------------------------------------------------------------------
// write(): send commanded position to each driver
// - called periodically by ros2_control
// -----------------------------------------------------------------------------

hardware_interface::return_type
arm_hardware::ArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Do nothing if not active yet (controllers may still be configuring)
  if (!active_) return hardware_interface::return_type::OK;

  if (drivers_.empty() || drivers_.size() != cmd_position_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
                 "write(): drivers/cmd buffers not initialized.");
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < drivers_.size(); ++i) {
    auto &drv = drivers_[i];
    if (!drv) {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                  "write(): driver[%zu] is null", i);
      return hardware_interface::return_type::ERROR;
    }

    // Command absolute position (already in rescaled units)
    if (!drv->write(cmd_position_[i])) {  // +10.0 is an offset for testing
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                  "write(): driver[%zu]->write failed: %s",
                  i, drv->last_error().c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  //RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "write(): cmd[0]=%.10f cmd[1]=%.10f ", cmd_position_[0], cmd_position_[1]);


  return hardware_interface::return_type::OK;
}

} // namespace arm_hardware


// The hardware interface written in c above will be loaded by the controller manager
// at runtime because we will build is as a plug-in. The controller manager will load
// all controllers and the hardware interface at runtime.
// We need to set the ArmHardwareInterface class as a plugin to be loaded.
// The header files need not be changed but the cpp file needs to have the following
// lines at the end of the file.

// Start with an include 

#include "pluginlib/class_list_macros.hpp"

// Macro code, include namespace and class name and then the second argument is the parent class we inherit from.

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)  
