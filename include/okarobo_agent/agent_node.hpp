#ifndef __AGENT_NODE_HPP__
#define __AGENT_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <agent_msgs/msg/agent.hpp>
#include <okarobo_msgs/msg/sensor.hpp>
#include <mpu9250/msg/sensor.hpp>

const int DIM = 3;

namespace OkaRobo{
class RoboAgent : public rclcpp::Node{
private:
    rclcpp::Subscription<okarobo_msgs::msg::Sensor>::SharedPtr robosense_sub_;
    rclcpp::Subscription<mpu9250::msg::Sensor>::SharedPtr mpu9250_sub_;
    rclcpp::Publisher<agent_msgs::msg::Agent>::SharedPtr agent_pub_;
    void _robosense_callback(const okarobo_msgs::msg::Sensor::SharedPtr RoboSense);
    void _mpu9250_callback(const mpu9250::msg::Sensor::SharedPtr IMU);
    void _agent_callback();
    rclcpp::TimerBase::SharedPtr agent_timer_;


    enum BallcdDir{
        Front,
        FrontLeft,
        Left,
        RearLeft,
        Rear,
        RearRight,
        Right,
        FrontRight,
        None
    };

    BallDir balldirection();

    float pi;

    uint16_t uss[2];
    uint16_t pre_uss[2];
    uint16_t bs[4];
    uint16_t pre_bs[4];

    float acc[3], rot[3], mag[3];
    float pre_acc[3], pre_rot[3], pre_mag[3];
    float deg2d, pre_deg2d;

    int16_t velocity, omega;
    int16_t nowAngle, targetAngle;

public:
    RoboAgent(
        const std::string& name_space="",
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};

template<typename TYPE, std::size_t SIZE>
std::size_t array_len(const TYPE (&array)[SIZE])
{
    return SIZE;
}


}


#endif