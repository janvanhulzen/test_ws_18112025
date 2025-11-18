#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "xl330_driver.hpp"

namespace gripper_hardware {

class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<XL330Driver> driver_;
    int joint1_motor_id_;
    std::string port_;
    // e.g., in gripper_hardware_interface.hpp
    double pos_moveit_{0.0};     // latest measured MoveIt-space position
    double cmd_moveit_{0.0};     // command in MoveIt-space
    double last_cmd_moveit_{0.0};
    std::string joint_name_{"axis_6"}; // make sure matches URDF/controller


}; // class GripperHardwareInterface

} // namespace gripper_hardware

#endif