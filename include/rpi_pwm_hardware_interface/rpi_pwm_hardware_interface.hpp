// Copyright (c) 2023, Atticus Russell
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef RPI_PWM_HARDWARE_INTERFACE__RPI_PWM_HARDWARE_INTERFACE_HPP_
#define RPI_PWM_HARDWARE_INTERFACE__RPI_PWM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rpi_pwm_hardware_interface/visibility_control.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rpi_pwm_hardware_interface/angular_servo.hpp"

namespace rpi_pwm_hardware_interface
{
class RPiPWMHardwareInterface : public hardware_interface::SystemInterface
{

struct ServoConfig
{
  std::string name = "";
  int pin = 0;
  int pi = 0;
  float min_angle = 0.0;
  float max_angle = 0.0;
  int min_pulse_width_us = 0;
  int max_pulse_width_us = 0;
};

struct ServoJoint
{
  std::string name = "";
  std::unique_ptr<AngularServo> servo;
  double pos = 0;
  double cmd = 0;
};

public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  ServoConfig srv_cfg_;
  ServoJoint rudder_joint_;
};

}  // namespace rpi_pwm_hardware_interface

#endif  // RPI_PWM_HARDWARE_INTERFACE__RPI_PWM_HARDWARE_INTERFACE_HPP_
