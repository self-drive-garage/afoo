
#include "afoo/actuators/afoo_actuator.hpp"

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

namespace afoo::actuators {

hardware_interface::CallbackReturn AfooActuator::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  hw_start_sec_ = hardware_interface::stod(
      info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = hardware_interface::stod(
      info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code
  hw_positions_.resize(info_.joints.size(),
                       std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("AfooActuator"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("AfooActuator"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("AfooActuator"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("AfooActuator"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("AfooActuator"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AfooActuator::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AfooActuator::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AfooActuator::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"),
               "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"), "%.1f seconds left...",
                 hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AfooActuator::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"),
               "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"), "%.1f seconds left...",
                 hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AfooActuator::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  for (std::size_t i = 0; i < hw_velocities_.size(); i++) {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"),
                 "Got position state %.5f and velocity state %.5f for '%s'!",
                 hw_positions_[i], hw_velocities_[i],
                 info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type afoo::actuators ::AfooActuator::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to
  // your production code
  RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"), "Writing...");

  // for (auto i = 0u; i < hw_commands_.size(); i++) {
  //   // Simulate sending commands to the hardware
  //   RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"),
  //               ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Got command %.5f for '%s'!",
  //               hw_commands_[i], info_.joints[i].name.c_str());

  //   hw_velocities_[i] = hw_commands_[i];
  // }

  bool logMessage = false;
  if (hw_commands_[0] != leftMotorSpeed_ ||
      hw_commands_[1] != rightMotorSpeed_) {
    logMessage = true;
  }

  leftMotorSpeed_ = hw_commands_[0];
  rightMotorSpeed_ = hw_commands_[1];

  hw_velocities_[0] = hw_commands_[0];
  hw_velocities_[1] = hw_commands_[1];

  leftMotorSpeed_ = std::clamp(leftMotorSpeed_, -127.0, 127.0);
  rightMotorSpeed_ = std::clamp(rightMotorSpeed_, -127.0, 127.0);

  if (is_between(leftMotorSpeed_, -3, 3) &&
      is_between(rightMotorSpeed_, -3, 3)) {
    // logMessage = true;
    leftMotorSpeed_ *= 50;
    rightMotorSpeed_ *= 50;
  }

  if (logMessage) {
    RCLCPP_INFO(rclcpp::get_logger("AfooActuator"),
                ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Got command %.5f for '%s'!",
                hw_commands_[0], info_.joints[0].name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("AfooActuator"),
                ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Got command %.5f for '%s'!",
                hw_commands_[1], info_.joints[1].name.c_str());

    RCLCPP_INFO(rclcpp::get_logger("AfooActuator"),
                "Clamping speed to %.5f for '%s'!", leftMotorSpeed_,
                info_.joints[0].name.c_str());

    RCLCPP_INFO(rclcpp::get_logger("AfooActuator"),
                "Clamping speed to %.5f for '%s'!", rightMotorSpeed_,
                info_.joints[1].name.c_str());
  }

  motorController_.driveMotor(1,
                              std::abs(static_cast<int>(1.5 * leftMotorSpeed_)),
                              hw_commands_[0] > 0, logMessage);
  motorController_.driveMotor(
      2, std::abs(static_cast<int>(1.5 * rightMotorSpeed_)),
      hw_commands_[1] > 0, logMessage);

  RCLCPP_DEBUG(rclcpp::get_logger("AfooActuator"),
               "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  return hardware_interface::return_type::OK;
}

}  // namespace afoo::actuators

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(afoo::actuators::AfooActuator,
                       hardware_interface::SystemInterface)
