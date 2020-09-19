#ifndef __AGENT_NODE_HPP__
#define __AGENT_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <agent_msgs/msg/agent.hpp>
#include <okarobo_msgs/msg/sensor.hpp>
#include <mpu9250/msg/sensor.hpp>

namespace OkaRobo{
class RoboSubPub : public rclcpp::Node{
private:
    rclcpp::Subscription<okarobo_msgs::msg::Sensor>::SharedPtr robosence_sub_;
    rclcpp::Subscription<mpu9250::msg::Sensor>::SharedPtr mpu9250_sub_;
    void _robosence_callback();
    void _mpu9250_callback();
    void _agent_callback();
    rclcpp::TimerBase::SharedPtr anget_timer_;

    uint16_t uss[2];
    uint16_t pre_uss[2];
    uint16_t bs[4];
    uint16_t pre_bs[4];

    float acc[3], rot[3], mag[3];
    float pre_acc[3], pre_rot[3], pre_mag[3];
    float deg, pre_deg;

    int16_t velocity, omega;
    int16_t nowAngle, targetAngle;

}


}

#endif