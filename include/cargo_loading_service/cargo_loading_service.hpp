// Copyright 2023 Tier IV, Inc.
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

#ifndef CARGO_LOADING_SERVICE__CARGO_LOADING_SERVICE_HPP_
#define CARGO_LOADING_SERVICE__CARGO_LOADING_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tier4_api_utils/tier4_api_utils.hpp"

#include "in_parking_msgs/msg/in_parking_status.hpp"
#include "in_parking_msgs/srv/execute_in_parking_task.hpp"
#include "v2i_interface_msgs/msg/infrastructure_command.hpp"
#include "v2i_interface_msgs/msg/infrastructure_command_array.hpp"
#include "v2i_interface_msgs/msg/infrastructure_state.hpp"
#include "v2i_interface_msgs/msg/infrastructure_state_array.hpp"

#include <string>
#include <chrono>

namespace cargo_loading_service
{

class CargoLoadingService : public rclcpp::Node
{
public:
  explicit CargoLoadingService(const rclcpp::NodeOptions & options);

private:
  using ExecuteInParkingTask = in_parking_msgs::srv::ExecuteInParkingTask;
  using InfrastructureCommand = v2i_interface_msgs::msg::InfrastructureCommand;
  using InfrastructureCommandArray = v2i_interface_msgs::msg::InfrastructureCommandArray;
  using InfrastructureState = v2i_interface_msgs::msg::InfrastructureState;
  using InfrastructureStateArray = v2i_interface_msgs::msg::InfrastructureStateArray;
  using InParkingStatus = in_parking_msgs::msg::InParkingStatus;

  // constants
  enum class CommandState : uint8_t { REQUESTING = 0b01, ERROR = 0b10 };
  enum class ReceiveState : uint8_t { APPROVAL = 0b01 };
  enum class InfraIdLimit : uint8_t { MIN = 1, MAX = 254 };
  static constexpr double timeout_time_{0.2};
  static constexpr double timeout_check_hz_{10};

  // variable
  uint8_t infra_id_;
  int32_t aw_state_{InParkingStatus::NONE};
  int32_t vehicle_operation_mode_{InParkingStatus::VEHICLE_MANUAL};
  bool infra_approval_{false};
  uint8_t service_result_{ExecuteInParkingTask::Response::NONE};
  double command_pub_hz_;
  double post_processing_time_;
  bool aw_state_timeout_{false};
  rclcpp::Time aw_state_last_receive_time_{rclcpp::Time(0)};

  // Service
  tier4_api_utils::Service<ExecuteInParkingTask>::SharedPtr srv_cargo_loading_;

  // Publisher
  rclcpp::Publisher<InfrastructureCommandArray>::SharedPtr pub_commands_;

  // Subscriber
  rclcpp::Subscription<InParkingStatus>::SharedPtr sub_inparking_status_;
  rclcpp::Subscription<InfrastructureStateArray>::SharedPtr sub_infrastructure_status_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timeout_check_timer_;

  // Callback
  void execCargoLoading(
    const ExecuteInParkingTask::Request::SharedPtr request,
    const ExecuteInParkingTask::Response::SharedPtr response);
  void onInParkingStatus(const InParkingStatus::ConstSharedPtr msg_ptr);
  void onInfrastructureStatus(const InfrastructureStateArray::ConstSharedPtr msg_ptr);
  void onTimer();
  void onTimeoutCheckTimer();
  void publishCommand(const uint8_t state);

  // Callback Group
  rclcpp::CallbackGroup::SharedPtr callback_group_subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
};

}  // namespace cargo_loading_service

#endif  // CARGO_LOADING_SERVICE__CARGO_LOADING_SERVICE_HPP_
