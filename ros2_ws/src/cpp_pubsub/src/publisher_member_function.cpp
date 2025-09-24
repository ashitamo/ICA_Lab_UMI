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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

void timer_callback(const rclcpp::Node::SharedPtr node, const rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher)
{
  static size_t count_ = 0;
  auto message = tutorial_interfaces::msg::Num();
  message.num = int64_t(count_++);
  RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", message.num);
  publisher->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
  auto timer = node->create_wall_timer(
    500ms, 
    static_cast<std::function<void()>>(std::bind(timer_callback, node, publisher))
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
