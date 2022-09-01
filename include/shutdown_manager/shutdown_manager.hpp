// Copyright 2022 eve autonomy inc. All Rights Reserved.
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
// limitations under the License

#ifndef SHUTDOWN_MANAGER__SHUTDOWN_MANAGER_HPP_
#define SHUTDOWN_MANAGER__SHUTDOWN_MANAGER_HPP_

#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <list>
#include <vector>
#include <climits>

#include "rclcpp/rclcpp.hpp"

#include "autoware_state_machine_msgs/msg/vehicle_button.hpp"
#include "shutdown_manager_msgs/msg/state_shutdown.hpp"

namespace shutdown_manager
{
using autoware_state_machine_msgs::msg::VehicleButton;
using shutdown_manager_msgs::msg::StateShutdown;

class ShutdownManager : public rclcpp::Node
{
public:
  explicit ShutdownManager(const rclcpp::NodeOptions & options);

private:

  uint16_t current_state_;
  float time_required_to_release_button_;
  float timeout_period_before_shutdown_aborts_;
  builtin_interfaces::msg::Time time_to_start_pressing_button_;
  bool button_data_;
  float button_hold_down_time_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher
  rclcpp::Publisher<StateShutdown>::SharedPtr pub_shutdown_state_;

  // Subscription
  rclcpp::Subscription<VehicleButton>::SharedPtr sub_shutdown_button_;

  void onShutdownButton(const VehicleButton::ConstSharedPtr msg);
  void publishShutdownState(const uint16_t msg_state);
  void onTimer(void);
};

}  // namespace shutdown_manager
#endif  // SHUTDOWN_MANAGER__SHUTDOWN_MANAGER_HPP_
