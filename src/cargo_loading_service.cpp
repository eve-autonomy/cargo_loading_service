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

#include "cargo_loading_service/cargo_loading_service.hpp"

#include <chrono>
#include <memory>

namespace cargo_loading_service
{

CargoLoadingService::CargoLoadingService(const rclcpp::NodeOptions & options)
: Node("cargo_loading_service", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);

  // Parameter
  command_pub_hz_ = this->declare_parameter("cargo_loadging_command_pub_hz", 5.0);

  // Callback group
  callback_group_service_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_subscription_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscribe_option = rclcpp::SubscriptionOptions();
  subscribe_option.callback_group = callback_group_subscription_;

  // Service
  srv_cargo_loading_ = proxy.create_service<ExecuteInParkingTask>(
    "/parking/cargo_loading", std::bind(&CargoLoadingService::execCargoLoading, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_service_);

  // Publisher
  pub_cargo_loading_state_ = this->create_publisher<InfrastructureCommandArray>(
    "/cargo_loading/infrastructure_commands", rclcpp::QoS{3}.transient_local());

  // Subscriber
  sub_inparking_status_ = this->create_subscription<InParkingStatus>(
    "/in_parking/state", rclcpp::QoS{1},
    std::bind(&CargoLoadingService::onInParkingState, this, _1), subscribe_option);
  sub_cargo_loading_state_ = this->create_subscription<InfrastructureStateArray>(
    "/infrastructure_status", rclcpp::QoS{1},
    std::bind(&CargoLoadingService::onCargoLoadingState, this, _1), subscribe_option);
}

void CargoLoadingService::execCargoLoading(
  const ExecuteInParkingTask::Request::SharedPtr request,
  const ExecuteInParkingTask::Response::SharedPtr response)
{
  using ExecuteInParkingTaskResponse = in_parking_msgs::srv::ExecuteInParkingTask::Response;

  const auto finalizing_pub_limit = static_cast<int32_t>(command_pub_hz_ * 2.0);
  int32_t finalizing_pub_count = 0;

  facility_id_ = request->value;

  response->state = ExecuteInParkingTaskResponse::SUCCESS;

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / command_pub_hz_));
  rclcpp::TimerBase::SharedPtr timer = create_timer(
    this, get_clock(), period,
    [&]() {
      InfrastructureCommand command;
      command.type = CMD_TYPE;
      command.id = request->value;

      if (finalize_) {
        command.state = InfrastructureCommand::SEND_ZERO;
      } else if (aw_state_ != InParkingStatus::NONE) {
        command.state = getCommandState();
      } else {
        response->state = ExecuteInParkingTaskResponse::FAIL;
        timer->cancel();
        return;
      }
      InfrastructureCommandArray command_array;
      auto stamp = this->get_clock()->now();
      command.stamp = stamp;
      command_array.commands.push_back(command);
      command_array.stamp = stamp;
      pub_cargo_loading_state_->publish(command_array);

      if (finalize_) {
        if (finalizing_pub_limit > finalizing_pub_count) {
          finalizing_pub_count++;
        } else {
          timer->cancel();
        }
      }
    },
    callback_group_subscription_);

  while (!timer->is_canceled()) {
    rclcpp::sleep_for(period);
  }

  // reinitialize
  facility_id_ = "";
  finalize_ = false;
}

uint8_t CargoLoadingService::getCommandState()
{
  uint8_t command_state = InfrastructureCommand::SEND_ZERO;

  if (aw_state_ == InParkingStatus::AW_EMERGENCY) {
    command_state = static_cast<std::underlying_type<CMD_STATE>::type>(CMD_STATE::ERROR);
  } else if (
    aw_state_ == InParkingStatus::AW_OUT_OF_PARKING ||
    aw_state_ == InParkingStatus::AW_UNAVAILABLE) {
    command_state = InfrastructureCommand::SEND_ZERO;
    if (!finalize_) {
      finalize_ = true;
    }
  } else {
    command_state = static_cast<std::underlying_type<CMD_STATE>::type>(CMD_STATE::REQUESTING);
  }

  return command_state;
}

void CargoLoadingService::onInParkingState(const InParkingStatus::ConstSharedPtr msg)
{
  aw_state_ = msg->aw_state;
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 0.05, "Subscribed /in_parking/state:%s",
    rosidl_generator_traits::to_yaml(*msg).c_str());
}

void CargoLoadingService::onCargoLoadingState(const InfrastructureStateArray::ConstSharedPtr msg)
{
  for (const auto & state : msg->states) {
    if (
      state.id.compare(facility_id_) == 0 && state.approval &&
      aw_state_ != InParkingStatus::AW_EMERGENCY) {
      finalize_ = true;
    }
  }
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 0.05, "Subscribed /infrastructure_status:%s",
    rosidl_generator_traits::to_yaml(*msg).c_str());
}
}  // namespace cargo_loading_service

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cargo_loading_service::CargoLoadingService)
