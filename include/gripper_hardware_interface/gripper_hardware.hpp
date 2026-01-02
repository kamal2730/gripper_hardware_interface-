#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "motion_sdk/usb_serial.hpp"
#include "motion_sdk/servo.hpp"

namespace gripper_hardware_interface
{

class GripperHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ROS-side states (meters, meters/sec)
  double position_state_{0.0};
  double velocity_state_{0.0};
  double effort_state_{0.0};

  // ROS-side command (meters)
  double position_command_{0.0};

  // Parameters
  std::string port_;
  int servo_id_{0};

  // SDK
  std::unique_ptr<motion_sdk::USBSerial> serial_;
  std::unique_ptr<motion_sdk::Servo> servo_;

  rclcpp::Logger logger_{rclcpp::get_logger("gripper_hardware")};

  // Mapping helpers
  double meters_to_degrees(double meters) const;
  double degrees_to_meters(double degrees) const;
};

}  // namespace gripper_hardware_interface
