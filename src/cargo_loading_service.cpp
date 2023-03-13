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
  command_pub_hz_ = this->declare_parameter<double>("command_pub_hz", 5.0);
  post_processing_time_ = this->declare_parameter<double>("post_processing_time", 2.0);

  // Callback group
  callback_group_subscription_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscribe_option = rclcpp::SubscriptionOptions();
  subscribe_option.callback_group = callback_group_subscription_;
  callback_group_service_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Service
  srv_cargo_loading_ = proxy.create_service<ExecuteInParkingTask>(
    "/in_parking/task", std::bind(&CargoLoadingService::execCargoLoading, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_service_);

  // Publisher
  pub_commands_ = this->create_publisher<InfrastructureCommandArray>(
    "/cargo_loading/infrastructure_commands", 10);

  // Subscriber
  sub_inparking_status_ = this->create_subscription<InParkingStatus>(
    "/in_parking/state", rclcpp::QoS{1},
    std::bind(&CargoLoadingService::onInParkingStatus, this, _1), subscribe_option);
  sub_infrastructure_status_ = this->create_subscription<InfrastructureStateArray>(
    "/v2i/infrastructure_states", rclcpp::QoS{1},
    std::bind(&CargoLoadingService::onInfrastructureStatus, this, _1), subscribe_option);

  // timer
  const auto period_ns = rclcpp::Rate(command_pub_hz_).period();
  const auto timeout_check_period_ns = rclcpp::Rate(timeout_check_hz_).period();
  timer_ = create_timer(
    this, get_clock(), period_ns, std::bind(&CargoLoadingService::onTimer, this),
    callback_group_subscription_);
  timeout_check_timer_ = create_timer(
    this, get_clock(), timeout_check_period_ns, std::bind(&CargoLoadingService::onTimeoutCheckTimer, this),
    callback_group_subscription_);

  // サービスcall時にtimerが回るように、最初にキャンセルしておく
  timer_->cancel();
}

void CargoLoadingService::execCargoLoading(
  const ExecuteInParkingTask::Request::SharedPtr request,
  const ExecuteInParkingTask::Response::SharedPtr response)
{
  if (request->id < static_cast<uint8_t>(InfraIdLimit::MIN) ||
      request->id > static_cast<uint8_t>(InfraIdLimit::MAX)) {
      RCLCPP_WARN(this->get_logger(), "Invalid ID = %d", request->id);
      response->state = ExecuteInParkingTask::Response::FAIL;
      return;
  }
  // 設備ID取得
  infra_id_ = request->id;
  service_result_ = ExecuteInParkingTask::Response::SUCCESS;

  // 設備連携要求開始
  if (timer_->is_canceled()) {
    timer_->reset();
    RCLCPP_DEBUG(this->get_logger(), "Timer restart");
  }

  // キャンセルになるまで設備連携要求を投げ続ける
  while (!timer_->is_canceled()) {
    rclcpp::sleep_for(rclcpp::Rate(command_pub_hz_).period());
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "request is running");
  }

  // サービスのresult更新
  response->state = service_result_;

  // 各変数の初期化
  service_result_ = ExecuteInParkingTask::Response::NONE;
  infra_id_ = InfrastructureState::INVALID_ID;
}

void CargoLoadingService::publishCommand(const uint8_t state)
{
  InfrastructureCommandArray command_array;
  auto stamp = this->get_clock()->now();
  command_array.stamp = stamp;

  InfrastructureCommand command;
  command.stamp = stamp;
  command.id = infra_id_;
  command.state = state;

  command_array.commands.push_back(command);
  pub_commands_->publish(command_array);
}

void CargoLoadingService::onTimer()
{
  // 設備連携が完了していない
  if (!infra_approval_) {
    // aw_stateで条件分岐
    switch (aw_state_) {
      // AWがEmergencyの場合はERRORを発出し続ける
      case InParkingStatus::AW_EMERGENCY:
        publishCommand(static_cast<std::underlying_type<CommandState>::type>(CommandState::ERROR));
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000 /* ms */, "AW emergency");
        break;
      // AWが停留所外などではinfra_approvalをtrueにし、設備連携結果はFAILで返す
      case InParkingStatus::AW_OUT_OF_PARKING:
      case InParkingStatus::AW_UNAVAILABLE:
      case InParkingStatus::NONE:
        RCLCPP_INFO(this->get_logger(), "AW_OUT_OF_PARKING or AW_UNAVAILABLE or NONE");
        infra_approval_ = true;
        service_result_ = ExecuteInParkingTask::Response::FAIL;
        break;
      // AWが停留所にいる場合
      case InParkingStatus::AW_WAITING_FOR_ROUTE:
      case InParkingStatus::AW_WAITING_FOR_ENGAGE:
      case InParkingStatus::AW_ARRIVED_PARKING:
        // 車両が自動モードなら設備連携要求を発出
        if (vehicle_operation_mode_ == InParkingStatus::VEHICLE_AUTO) {
          RCLCPP_DEBUG(this->get_logger(), "requesting");
          publishCommand(
          static_cast<std::underlying_type<CommandState>::type>(CommandState::REQUESTING));
        } else {  // 車両が手動モードならinfra_approvalをtrueにし、設備連携結果はFAILで返す
          RCLCPP_INFO(this->get_logger(), "vehicle is manual mode");
          infra_approval_ = true;
          service_result_ = ExecuteInParkingTask::Response::FAIL;
        }
        break;
      default:
        break;
    }
  } else {  // 設備連携が完了
    RCLCPP_INFO(this->get_logger(), "try reporting to infrastructure that the cargo loading process is over.");
    // SEND_ZEROをn秒間発出し、設備連携結果はSUCCESSで返し、timerをキャンセル
    const auto start_time = this->now();
    while (true) {
      publishCommand(InfrastructureCommand::SEND_ZERO);
      const auto time_diff = this->now() - start_time;
      if (time_diff.seconds() > post_processing_time_) break;
      rclcpp::sleep_for(rclcpp::Rate(command_pub_hz_).period());
    }
    RCLCPP_INFO(this->get_logger(), "complete reporting to infrastructure that the cargo loading process is over.");
    infra_approval_ = false;
    infra_id_ = InfrastructureState::INVALID_ID;
    timer_->cancel();
  }
}

void CargoLoadingService::onTimeoutCheckTimer()
{
  if (timer_->is_canceled() || aw_state_timeout_) {
    return;
  }

  auto receive_time_diff = get_clock()->now() - aw_state_last_receive_time_;
  if (receive_time_diff.seconds() > timeout_time_) {
    aw_state_ = InParkingStatus::AW_EMERGENCY;
    sub_inparking_status_ = nullptr;
    aw_state_timeout_ = true;
    RCLCPP_ERROR(
      this->get_logger(), "/in_parking/state receive timeout. Last received time (seconds) = %lf",
      aw_state_last_receive_time_.seconds());
  }
}

void CargoLoadingService::onInParkingStatus(const InParkingStatus::ConstSharedPtr msg)
{
  aw_state_last_receive_time_ = msg->stamp;
  aw_state_ = msg->aw_state;
  vehicle_operation_mode_ = msg->vehicle_operation_mode;

  if (msg->aw_state == InParkingStatus::AW_EMERGENCY) {
    sub_inparking_status_ = nullptr;
  }

  RCLCPP_DEBUG(
    this->get_logger(), "inParkingStatus: %s", rosidl_generator_traits::to_yaml(*msg).c_str());
}

void CargoLoadingService::onInfrastructureStatus(const InfrastructureStateArray::ConstSharedPtr msg)
{
  const auto itr = std::find_if(
    msg->states.begin(), msg->states.end(), [this](const auto & e) { return e.id == infra_id_; });

  if (itr != msg->states.end()) {
    const auto & e = msg->states.at(std::distance(msg->states.begin(), itr));

    // 成功した場合、APPROVALが返ってくる
    infra_approval_ = (e.state == static_cast<uint8_t>(ReceiveState::APPROVAL));

    // APPROVALじゃない場合、エラー出力
    if (e.state != static_cast<uint8_t>(ReceiveState::APPROVAL)) {
      RCLCPP_ERROR(this->get_logger(), "invalid return value: %d", e.state);
    }
  }

  RCLCPP_DEBUG(
    this->get_logger(), "InfrastructureStatus: %s", rosidl_generator_traits::to_yaml(*msg).c_str());
}
}  // namespace cargo_loading_service

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cargo_loading_service::CargoLoadingService)
