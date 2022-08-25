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


#include <memory>
#include <utility>
#include "shutdown_manager/shutdown_manager.hpp"

namespace shutdown_manager
{

ShutdownManager::ShutdownManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("shutdown_manager", options)
{
  using std::placeholders::_1;

  sub_shutdown_button_ = this->create_subscription<VehicleButton>(
    "input/shutdown_button", rclcpp::QoS{1}.transient_local(),
    std::bind(&ShutdownManager::onShutdownButton, this, _1));

  pub_delivery_reservation_state_ = this->create_publisher<StateLock>(
    "output/lock_state", rclcpp::QoS{3}.transient_local());

  time_required_to_release_button_ = this->declare_parameter<float>(
    "time_required_to_release_button", 5.0);
  timeout_period_before_shutdown_aborts_ = this->declare_parameter<float>(
    "timeout_period_before_shutdown_aborts", 10.0);

  std::chrono::milliseconds timer_period_msec;
  timer_period_msec = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / 5.0));

  auto timer_callback = std::bind(&ShutdownManager::onTimer, this);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), timer_period_msec, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void ShutdownManager::onShutdownButton(const VehicleButton::ConstSharedPtr msg)
{
  switch (current_state_) {
    case NodeStatus::WAITING_FOR_BUTTON_PRESS:
      if (msg->hold_down_time >= time_required_to_release_button_) {
        time_to_start_pressing_button_ = msg->stamp;
        publishReservationLampState(StateLock::STATE_STANDBY_FOR_SHUTDOWN);
        current_state_ = NodeStatus::WAITING_FOR_BUTTON_RELEASE;
      }
      break;
    case NodeStatus::WAITING_FOR_BUTTON_RELEASE:
      if (msg->data) {
        current_state_ = NodeStatus::SHUTDOWN_RECEPTION;
      }
      break;
    case NodeStatus::SHUTDOWN_RECEPTION:
      if (msg->data) {
        publishReservationLampState(StateLock::STATE_START_OF_SHUTDOWN);
        const auto ret = system("systemctl poweroff");
        if (!WIFEXITED(ret) || WEXITSTATUS(ret) != 0) {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(), 1.0,
            "[shutdown_manager] System shutdown ERROR!!! ");
          publishReservationLampState(StateLock::STATE_STANDBY_FOR_SHUTDOWN);
        } else {
          current_state_ = NodeStatus::SHUTDOWN_IN_PROGRESS;
        }
      }
      break;
    default:
      break;
  }
}

void ShutdownManager::publishReservationLampState(const uint16_t msg_state)
{
  StateLock msg;
  msg.stamp = this->now();
  msg.state = msg_state;
  pub_delivery_reservation_state_->publish(msg);
}

void ShutdownManager::onTimer(void)
{
  if ((current_state_ != NodeStatus::WAITING_FOR_BUTTON_RELEASE) &&
    (current_state_ != NodeStatus::SHUTDOWN_RECEPTION))
  {
    return;
  }
  const auto time_diff = this->now() - time_to_start_pressing_button_;
  if (time_diff.seconds() > timeout_period_before_shutdown_aborts_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1.0,
      "[shutdown_manager] Timeout for shutdown standby ");
    publishReservationLampState(StateLock::STATE_INACTIVE_FOR_SHUTDOWN);
    current_state_ = NodeStatus::WAITING_FOR_BUTTON_PRESS;
  }
}

}  // namespace shutdown_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(shutdown_manager::ShutdownManager)
