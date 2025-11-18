#include "gripper_hardware_interface.hpp"
#include <algorithm>  // for std::max / std::min

namespace {  // anonymous namespace = file-local linkage

// Calibration constants and helper functions
constexpr double A = -143.67924528301887;
constexpr double B = -0.1785188679245282;

inline double moveit_to_dyn(double x) { return A * x + B; }
inline double dyn_to_moveit(double y) { return (y - B) / A; }

constexpr double MOVEIT_MIN = -0.0121;
constexpr double MOVEIT_MAX =  0.0091;
constexpr double DYN_MIN    = -1.60;
constexpr double DYN_MAX    =  1.60;

inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

}  // end anonymous namespace

namespace gripper_hardware {

hardware_interface::CallbackReturn GripperHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    joint1_motor_id_ = std::stoi(info_.hardware_parameters["joint1_motor_id"]);
    port_ = info_.hardware_parameters["dynamixel_port"];

    driver_ = std::make_shared<XL330Driver>(port_);

    RCLCPP_INFO(get_logger(), "On init of gripper with ID: %i", joint1_motor_id_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() !=0) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->activateWithPositionMode(joint1_motor_id_);

    const double dyn_now = driver_->getPositionRadian(joint1_motor_id_);
    const double moveit_now = clamp(dyn_to_moveit(dyn_now), MOVEIT_MIN, MOVEIT_MAX);

    // Initialize both state and command for axis_6
    //set_state("axis_6/position", moveit_now);
    //set_command("axis_6/position", moveit_now);   // <-- important

    RCLCPP_INFO(get_logger(), "Activated gripper: dyn=%.4f rad → moveit=%.6f",
                dyn_now, moveit_now);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->deactivate(joint1_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    const double dyn_pos = clamp(driver_->getPositionRadian(joint1_motor_id_), DYN_MIN, DYN_MAX);
    pos_moveit_ = clamp(dyn_to_moveit(dyn_pos), MOVEIT_MIN, MOVEIT_MAX);
    set_state("axis6/position", pos_moveit_);

    RCLCPP_INFO(get_logger(), "set state axis_6 %lf",get_state("axis_6/position"));

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    double cmd_moveit = get_command("axis_6/position");

    // If the controller hasn't written yet, hold current state
    if (!std::isfinite(cmd_moveit)) {
        cmd_moveit = get_state("axis_6/position");  // or cache last good cmd
        // Optional: throttle the warning
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        //   "axis_6 command NaN; holding at %.6f", cmd_moveit);
    }

    cmd_moveit = clamp(cmd_moveit, MOVEIT_MIN, MOVEIT_MAX);
    const double dyn_cmd = clamp(moveit_to_dyn(cmd_moveit), DYN_MIN, DYN_MAX);

    driver_->setTargetPositionRadian(joint1_motor_id_, dyn_cmd);

    // Log AFTER coalescing, so you never print NaN
    RCLCPP_INFO(get_logger(), "CMD axis_6: moveit=%.6f → dyn=%.4f", cmd_moveit, dyn_cmd);


    return hardware_interface::return_type::OK;
}

} // namespace gripper_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gripper_hardware::GripperHardwareInterface, hardware_interface::SystemInterface)
