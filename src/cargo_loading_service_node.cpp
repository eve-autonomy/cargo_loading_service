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

#include <memory>

#include "cargo_loading_service/cargo_loading_service.hpp"

int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  rclcpp::ExecutorOptions exe_options = rclcpp::ExecutorOptions();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<cargo_loading_service::CargoLoadingService>(options);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exe_options, 2);
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();

  return 0;
}
