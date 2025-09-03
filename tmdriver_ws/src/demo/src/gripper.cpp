#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/set_positions.hpp"
#include "robotiq_85_msgs/msg/gripper_cmd.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

bool wait_for_service(const rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr& client) {
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "中斷等待 service，退出");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "等待 set_positions 服務中...");
    }
    return true;
}

bool is_valid_gripper(double v) {
    return v >= 0.0 && v <= 0.085;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gripper_control");

    auto arm_client = node->create_client<tm_msgs::srv::SetPositions>("set_positions");
    auto gripper_pub = node->create_publisher<robotiq_85_msgs::msg::GripperCmd>("/gripper/cmd", 10);

    // ✅ 初始座標
    std::vector<double> current_pose = {0.2, -0.342, 0.35, 3.14159, 0.0, -1.57};

    auto init_req = std::make_shared<tm_msgs::srv::SetPositions::Request>();
    init_req->motion_type = tm_msgs::srv::SetPositions::Request::PTP_T;
    init_req->positions = current_pose;
    init_req->velocity = 2.0;
    init_req->acc_time = 0.4;
    init_req->blend_percentage = 0;
    init_req->fine_goal = true;

    std::cout << "⚙️  移動到初始位置...\n";
    if (wait_for_service(arm_client)) {
        arm_client->async_send_request(init_req);
    }

    // 🟢 互動控制
    std::string line;
    std::cout << "🔧 輸入 7 個浮點數（x y z rx ry rz gripper_width），或輸入 3（只更新xyz）、1（只控制gripper），exit 離開\n";

    while (rclcpp::ok()) {
        std::cout << "輸入 > ";
        std::getline(std::cin, line);
        if (line == "exit") break;

        std::istringstream iss(line);
        std::vector<double> values;
        double num;
        while (iss >> num) values.push_back(num);

        if (values.size() == 7) {
            // arm + gripper
            std::vector<double> pos(values.begin(), values.begin() + 6);
            double grip = values[6];

            if (!is_valid_gripper(grip)) {
                std::cout << "❌ 夾爪開度需在 0.0 到 0.085 之間\n";
                continue;
            }

            current_pose = pos; // 更新全體姿態
            auto req = std::make_shared<tm_msgs::srv::SetPositions::Request>();
            req->motion_type = tm_msgs::srv::SetPositions::Request::PTP_T;
            req->positions = current_pose;
            req->velocity = 2;
            req->acc_time = 0.4;
            req->blend_percentage = 0;
            req->fine_goal = true;

            if (!wait_for_service(arm_client)) break;
            arm_client->async_send_request(req);
            std::cout << "✅ 已送出 arm 位置\n";

            robotiq_85_msgs::msg::GripperCmd grip_msg;
            grip_msg.emergency_release = false;
            grip_msg.emergency_release_dir = 0;
            grip_msg.stop = false;
            grip_msg.position = grip;
            grip_msg.speed = 0.1;
            grip_msg.force = 1.0;
            gripper_pub->publish(grip_msg);
            std::cout << "✅ 已送出 gripper 控制（" << grip << "）\n";
        }
        else if (values.size() == 3) {
            // 只更新 xyz
            current_pose[0] = values[0];
            current_pose[1] = values[1];
            current_pose[2] = values[2];

            auto req = std::make_shared<tm_msgs::srv::SetPositions::Request>();
            req->motion_type = tm_msgs::srv::SetPositions::Request::PTP_T;
            req->positions = current_pose;
            req->velocity = 2;
            req->acc_time = 0.4;
            req->blend_percentage = 0;
            req->fine_goal = true;

            if (!wait_for_service(arm_client)) break;
            arm_client->async_send_request(req);
            std::cout << "✅ 已更新 arm 前三個位置（xyz）為："
                      << values[0] << ", " << values[1] << ", " << values[2] << "\n";
        }
        else if (values.size() == 1) {
            // only gripper
            double grip = values[0];
            if (!is_valid_gripper(grip)) {
                std::cout << "❌ 夾爪開度需在 0.0 到 0.085 之間\n";
                continue;
            }

            robotiq_85_msgs::msg::GripperCmd grip_msg;
            grip_msg.emergency_release = false;
            grip_msg.emergency_release_dir = 0;
            grip_msg.stop = false;
            grip_msg.position = grip;
            grip_msg.speed = 0.1;
            grip_msg.force = 1.0;

            gripper_pub->publish(grip_msg);
            std::cout << "✅ 單獨送出 gripper 控制（" << grip << "）\n";
        }
        else {
            std::cout << "❌ 輸入格式錯誤！請輸入 7、3 或 1 個浮點數。\n";
        }
    }

    rclcpp::shutdown();
    return 0;
}
