// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/diffdriveESP/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_esp_32
{
hardware_interface::CallbackReturn DiffDriveESP32HW::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  l_wheel_.setup(info_.joints[0].name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(info_.joints[1].name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    
    }
    bool has_position = false, has_velocity = false;
    for (auto &si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_position = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY) has_velocity = true;
    }
    if (!has_position || !has_velocity) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' missing required state interfaces",
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    // {
    //   RCLCPP_FATAL(
    //     get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
    //     joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
    //     hardware_interface::HW_IF_POSITION);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    // {
    //   RCLCPP_FATAL(
    //     get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
    //     joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
    //     hardware_interface::HW_IF_VELOCITY);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveESP32HW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  // for (int i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  // for (const auto & [name, descr] : joint_state_interfaces_)
  // {
  //   set_state(name, 0.0);
  // }
  // for (const auto & [name, descr] : joint_command_interfaces_)
  // {
  //   set_command(name, 0.0);
  // }
  // RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveESP32HW::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
    
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));

    for (auto &si : state_interfaces) { 
      RCLCPP_INFO(get_logger(), "Exported state interface: %s/%s",
            si.get_name().c_str(), si.get_interface_name().c_str());
    }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveESP32HW::export_command_interfaces(){
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveESP32HW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  // command and state should be equal when starting
  l_wheel_.cmd = 5.0;
  r_wheel_.cmd = 0.0;


  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveESP32HW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  // comms_.disconnect();
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveESP32HW::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // 1. read encoders from ESP32
  comms_.readEncoder(
      {l_wheel_.name, r_wheel_.name},
      l_wheel_.enc, r_wheel_.enc);

  double dt = period.seconds();

  // 2. compute tick deltas
  int dl = l_wheel_.enc - l_wheel_.enc_prev;
  int dr = r_wheel_.enc - r_wheel_.enc_prev;

  l_wheel_.enc_prev = l_wheel_.enc;
  r_wheel_.enc_prev = r_wheel_.enc;

  // 3. integrate position
  l_wheel_.pos += dl * l_wheel_.rads_per_count;
  r_wheel_.pos += dr * r_wheel_.rads_per_count;

  // 4. compute velocity
  if (dt > 0.0) {
    l_wheel_.vel = dl * l_wheel_.rads_per_count / dt;
    r_wheel_.vel = dr * r_wheel_.rads_per_count / dt;
  } else {
    l_wheel_.vel = 0.0;
    r_wheel_.vel = 0.0;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type DiffDriveESP32HW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  double l_motor_counts_per_loop = std::round(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate);
  double r_motor_counts_per_loop = std::round(r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

  comms_.setValue(l_wheel_.name, l_motor_counts_per_loop, r_wheel_.name, r_motor_counts_per_loop);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_esp_32

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_esp_32::DiffDriveESP32HW, hardware_interface::SystemInterface)
