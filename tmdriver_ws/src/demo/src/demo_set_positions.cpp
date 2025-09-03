#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/set_positions.hpp"
#include "robotiq_85_msgs/msg/gripper_cmd.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>


using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::vector<double> positions = {0.300, -0.300, 0.300, 3.14159, 0, -3.14159/2};
  std::scanf("%lf %lf %lf", &positions[0], &positions[1], &positions[2]);
  std::scanf("%lf %lf %lf", &positions[3], &positions[4], &positions[5]);
  std::cout << positions[0] << " " << positions[1] << " " << positions[2] << std::endl;
  std::cout << positions[3] << " " << positions[4] << " " << positions[5] << std::endl;
  // positions[3]= positions[3]/180.0*3.14159;
  // positions[4]= positions[4]/180.0*3.14159;
  // positions[5]= positions[5]/180.0*3.14159;

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_set_positions");
  rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr client =
    node->create_client<tm_msgs::srv::SetPositions>("set_positions");
  
  auto request = std::make_shared<tm_msgs::srv::SetPositions::Request>();
  request->motion_type = tm_msgs::srv::SetPositions::Request::PTP_T;
  request->positions = positions;
  request->velocity = 2;//rad/s
  request->acc_time = 2;
  request->blend_percentage = 0;
  request->fine_goal  = true;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    if(result.get()->ok){
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
    } else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
    }

  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  return true;

  rclcpp::shutdown();
  return 0;
}
