#include <rclcpp/rclcpp.hpp>
#include <agent_msgs/msg/agent.hpp>
#include <okarobo_msgs/msg/sensor.hpp>
#include <mpu9250/msg/sensor.hpp>

#include <cmath>

#include <okarobo_agent/agent_node.hpp>

namespace OkaRobo{
RoboAgent::RoboAgent(
    const std::string& name_space,
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
        std::bind(&RoboAgent::_agent_callback, this)
    );

}

void RoboAgent::_robosense_callback(const okarobo_msgs::msg::Sensor::SharedPtr RoboSense)
{
    for(auto i = 0; i < array_len(uss); i++){
        this->pre_uss[i] = this->uss[i];
    }
    for(auto i = 0; i < array_len(bs); i++){
        this->pre_bs[i] = this->bs[i];
    }

    this->uss[0] = RoboSense->ussl;
    this->uss[1] = RoboSense->ussr;
    this->bs[0] = RoboSense->bsfront;
    this->bs[1] = RoboSense->bsleft;
    this->bs[2] = RoboSense->bsright;
    this->bs[3] = RoboSense->bsrear;

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

    this->pre_deg2d = this->deg2d;
    this->deg2d = std::atan2(this->mag[0], this->mag[1]) * 180 / pi;
    this->nowAngle = this->deg2d;
}

void RoboAgent::_agent_callback()
{
    auto msg = std::make_shared<agent_msgs::msg::Agent>();

    std::string msg_dir;

    RCLCPP_INFO(this->get_logger(), "F %d, L %d, R %d, Re %d", bs[0], bs[1], bs[2], bs[3]);

    velocity = 250;
    BallDir dir = balldirection();

    switch(balldirection()){
        case Front:
            omega = 90;
            msg_dir = "Front";
            break;
        case FrontLeft:
            omega = 180;
            msg_dir = "FrontLeft";
            break;
        case Left:
            omega = 200;
            msg_dir = "Left";
            break;
        case RearLeft:
            omega = 270;
            msg_dir = "RearLeft";
            break;
        case Rear:{
            if(uss[0] < uss[1]){
                omega = 270 - 45;
            }
            else{
                omega = 270 + 45;
            }
            msg_dir = "Rear";
            break;
        }
        case RearRight:
            omega = 270;
            msg_dir = "RearRight";
            break;
        case Right:
            omega = 340;
            msg_dir = "Right";
            break;
        case FrontRight:
            omega = 0;
            msg_dir = "FrontRight";
            break;
        default:
            msg_dir = "None";
            velocity = 0;
            break;

    }

    if(dir == None){
        this->targetAngle = 0; 
    }
    
    msg->velocity = velocity;
    msg->omega = this->omega;
    msg->nowangle = this->nowAngle;
    msg->targetangle = this->targetAngle;

    RCLCPP_INFO(this->get_logger(), "velocity %d, omega %d, dir %s", velocity, omega, msg_dir.c_str());

    agent_pub_->publish(*msg);

}

RoboAgent::BallDir RoboAgent::balldirection()
{
    std::vector<uint16_t> bsc(std::begin(this->bs), std::end(this->bs));
    std::vector<uint16_t>::iterator min_first = std::min_element(bsc.begin(), bsc.end());   //minimum iterotar of bss
    auto min_first_index = std::distance(bsc.begin(), min_first);

    auto min_first_value = *min_first;

    if(*min_first < 500){
        *min_first = 1023;
        std::vector<uint16_t>::iterator min_sec = std::min_element(bsc.begin(), bsc.end());
        RCLCPP_INFO(this->get_logger(), "min index %d, min val %d", min_first_index, *min_first);
        if(*min_sec < 500){
            switch(min_first_index){
                case 0:{
                    auto min_sec_index = std::distance(bsc.begin(), min_sec);
                    if(min_sec_index == 1){
                        return FrontLeft;
                    }
                    else if(min_sec_index == 2){
                        return FrontRight;
                    }
                    return Front;
                }
                case 1:{
                    auto min_sec_index = std::distance(bsc.begin(), min_sec);
                    if(min_sec_index == 0){
                        return FrontLeft;
                    }
                    else if(min_sec_index == 3){
                        return RearLeft;
                    }
                    return Left;
                }
                case 2:{
                    auto min_sec_index = std::distance(bsc.begin(), min_sec);
                    if(min_sec_index == 0){
                        return FrontRight;
                    }
                    else if(min_sec_index == 3){
                        return RearRight;
                    }
                    return Right;
                }
                case 3:{
                    auto min_sec_index = std::distance(bsc.begin(), min_sec);
                    if(min_sec_index == 1){
                        return RearLeft;
                    }
                    else if(min_sec_index == 2){
                        return RearRight;
                    }
                    return Rear;
                }
            }
        }
        else if(min_first_value < 900){
            switch(min_first_index){
                case 0:
                    return Front;
                case 1:
                    return Left;
                case 2:
                    return Right;
                case 3:
                    return Rear;
            }
        }

    }
    else if(*min_first < 1000){
        switch(min_first_index){
            case 0:
                return Front;
            case 1:
                return Left;
            case 2:
                return Right;
            case 3:
                return Rear;
        }
    }
    else{
        return None;
    }
}

}
