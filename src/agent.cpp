#include <rclcpp/rclcpp.hpp>
#include <okarobo_agent/agent_node.hpp>

#include <chrono>
#include <thread>
#include <iostream>

using namespace std;
using namespace OkaRobo;

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoboAgent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}