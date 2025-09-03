#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/msg/feedback_state.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("demo_get_feedback")
    {
      subscription_ = this->create_subscription<tm_msgs::msg::FeedbackState>(
      "feedback_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    
    void topic_callback(const tm_msgs::msg::FeedbackState::SharedPtr msg) const
    {
      if(msg->tool0_pose.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"FeedbackState: tool0_pose = (" << 
                msg->tool0_pose[0] << ", " << 
                msg->tool0_pose[1] << ", " << 
                msg->tool0_pose[2] << ", " <<
                msg->tool0_pose[3] << ", " << 
                msg->tool0_pose[4] << ", " << 
                msg->tool0_pose[5] << ")"); 
      }
      if(msg->tool_pose.size() == 6){
        RCLCPP_INFO_STREAM(this->get_logger(),"FeedbackState: tool_pose = (" << 
                msg->tool_pose[0] << ", " << 
                msg->tool_pose[1] << ", " << 
                msg->tool_pose[2] << ", " <<
                msg->tool_pose[3] << ", " << 
                msg->tool_pose[4] << ", " << 
                msg->tool_pose[5] << ")"); 
      }
    }
    rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
