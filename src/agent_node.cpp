#include <rclcpp/rclcpp.hpp>
#include <agent_msgs/msg/agent.hpp>
#include <okarobo_msgs/msg/sensor.hpp>
#include <mpu9250/msg/sensor.hpp>

#include <cmath>

#include <okarobo_agent/agent_node.hpp>

namespace OkaRobo{
RoboAgent::RoboAgent(
    const std::string& name_space="",
    const rclcpp::NodeOptions& options
): Node("robo_agent", name_space, options)
{
    using namespace std::chrono_literals;

    pi = std::acos(-1);

    robosense_sub_ = this->create_subscription<okarobo_msgs::msg::Sensor>(
        "robo_sensor",
        rclcpp::QoS(10),
        std::bind(&RoboAgent::_robosense_callback, this, std::placeholders::_1)
    );

    mpu9250_sub_ = this->create_subscription<mpu9250::msg::Sensor>(
        "mpu9250",
        rclcpp::QoS(10),
        std::bind(&RoboAgent::_mpu9250_callback, this, std::placeholders::_1)
    );

    agent_pub_ = this->create_publisher<agent_msgs::msg::Agent>(
        "agent",
        rclcpp::QoS(10)
    );

    agent_timer_ = this->create_wall_timer(
        50ms,
        std::bind(&RoboAgent::_agent_callbakc, this)
    );

}

void RoboAgent::_robosense_callback(const okarobo_msgs::msg::Sensor::SharedPtr RoboSense)
{

}

void RoboAgent::_mpu9250_callback(const mpu9250::msg::Sensor::SharedPtr IMU)
{
    for(auto i = 0; i < DIM; i++){
        this->pre_acc[i] = this->acc[i];
        this->pre_rot[i] = this->rot[i];
        this->pre_mag[i] = this->mag[i];

        this->acc[i] = IMU->acc[i];
        this->rot[i] = IMU->rot[i];
        this->mag[i] = IMU->mag[i];
    }

    this->pre_deg2d = deg;
    this->deg = std::atan2(this->mag[0], this->mag[1]) * 180 / pi;
    this->nowAngle = deg;
}

void RoboAgent::_agent_callback()
{
    auto msg = std::make_shared<agent_msgs::msg::Agent>();
    msg->velocity = 0;
    msg->omega = 0;
    msg->nowangle = this->nowAngle;
    msg->targetangle = this->targetAngle;


}

}