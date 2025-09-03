#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

bool send_cmd(std::string cmd, std::shared_ptr<rclcpp::Node> node, rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client){
  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "demo";
  request->script = cmd;

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
}

  //PTP(\"CPP\",200.00000,-342.00000,350.00000,179.99985,0.00000,-89.95437,63,400,0,true)
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string cmds[] = {
    "PVTEnter(1)",
    "PVTPoint(467.5,-122.2,359.7,180,0,-90, 50,50,0,0,0,0, 0.2)" ,
    "QueueTag(100)",
    "WaitQueueTag(100)",
    "PVTPoint(467.5,-72.2,359.7,180,0,-90, -50,50,0,0,0,0, 0.2)",
    "PVTPoint(417.5,-72.2,359.7,180,0,-90,  0,0,0,0,0,0, 0.2)",
    "PVTPoint(417.5,-122.2,359.7,180,0,-90, 50,50,0,0,0,0, 0.2)",
    "PVTExit()"
  };
  // std::string cmds[] = {
  //   "string name = \"my_counter\" \r\n val = GetVarValue(name)\r\n ListenSend(90, val)",
  // };

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_send_script");
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client =
    node->create_client<tm_msgs::srv::SendScript>("send_script");
  for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
    send_cmd(cmds[i], node, client);
    rclcpp::sleep_for(1ms);
  }
  std::string cmd = "PTP(\"CPP\",200.00000,-342.00000,350.00000,179.99985,0.00000,-89.95437,63,400,0,true)";
  
  // send_cmd(cmd, node, client);

  rclcpp::shutdown();
  return 0;
}
