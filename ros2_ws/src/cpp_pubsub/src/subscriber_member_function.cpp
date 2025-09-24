// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"
using std::placeholders::_1;

void topic_callback(
  const rclcpp::Node::SharedPtr node,
  const tutorial_interfaces::msg::Num::SharedPtr msg)
  {
    RCLCPP_INFO(node->get_logger(), "I heard: '%d'", msg->num);
  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("minimal_subscriber");
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription;
  subscription = node->create_subscription<tutorial_interfaces::msg::Num>(
    "topic", 10, static_cast<std::function<void(const tutorial_interfaces::msg::Num::SharedPtr)>>(std::bind(topic_callback, node, _1))
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
