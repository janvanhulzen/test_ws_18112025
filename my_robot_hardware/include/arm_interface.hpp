#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

// Brings in the abstract base class hardware_interface::SystemInterface
#include "hardware_interface/system_interface.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "PhidgetsStepperDriver.hpp"  // provides armhw::StepperParams and armhw::StepperDriver

namespace arm_hardware {

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // lifecycle overrides
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // system interface overrides
  hardware_interface::return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // add overwrites for exporting interfaces

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


private:
  // ---- Per-joint configuration & runtime objects ----
  // Filled from YAML in on_init/on_configure
  std::vector<armhw::StepperParams> params_;                      // one per motor

  // Created in on_configure(), engaged in on_activate()
  std::vector<std::unique_ptr<armhw::StepperDriver>> drivers_;    // one per motor

  // ---- ros2_control command/state buffers (position interface) ----
  std::vector<double> cmd_position_;   // commands from controllers
  std::vector<double> state_position_; // measured position
  std::vector<double> state_velocity_; // measured velocity
  std::vector<double> state_moving_;   // 1.0 = moving, 0.0 = stopped

  bool configured_ {false};
  bool active_ {false};
};

} // namespace arm_hardware

#endif // ARM_HARDWARE_INTERFACE_HPP
