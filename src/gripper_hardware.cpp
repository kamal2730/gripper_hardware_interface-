#include "gripper_hardware_interface/gripper_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>

namespace gripper_hardware_interface
{

hardware_interface::CallbackReturn
GripperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters
  port_ = info.hardware_parameters.at("port");
  servo_id_ = std::stoi(info.hardware_parameters.at("id"));

  serial_ = std::make_unique<motion_sdk::USBSerial>(port_, 115200);

  if (!serial_->open())
  {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  servo_ = std::make_unique<motion_sdk::Servo>(*serial_, servo_id_);

  if (!servo_->ping())
  {
    RCLCPP_ERROR(logger_, "Servo ping failed (id=%d)", servo_id_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Gripper hardware initialized");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GripperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &position_state_);

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &velocity_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GripperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &position_command_);

  return command_interfaces;
}

hardware_interface::return_type
GripperHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  float angle_deg = 0.0f;

  if (!servo_->getPosition(angle_deg))
  {
    RCLCPP_WARN(logger_, "Failed to read gripper position");
    return hardware_interface::return_type::ERROR;
  }

  position_state_ = degrees_to_meters(angle_deg);
  velocity_state_ = 0.0;  // dummy velocity (required by controller)

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
GripperHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Clamp command: 0.0 m → 0.04 m
  position_command_ = std::clamp(position_command_, 0.0, 0.04);

  float angle_deg =
    static_cast<float>(meters_to_degrees(position_command_));

  if (!servo_->setPosition(angle_deg))
  {
    RCLCPP_WARN(logger_, "Failed to write gripper position");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

double GripperHardware::meters_to_degrees(double meters) const
{
  return (meters / 0.04) * 90.0;
}

double GripperHardware::degrees_to_meters(double degrees) const
{
  return (degrees / 90.0) * 0.04;
}

}  // namespace gripper_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  gripper_hardware_interface::GripperHardware,
  hardware_interface::SystemInterface
)
