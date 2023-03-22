// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>     // Date and time
#include <memory>     // Dynamic memory management
#include <functional>
#include <iostream> // input and output on terminal
#include <string>   // String functions
#include <cmath>

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"

// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"



using namespace std::placeholders;
using namespace std::chrono_literals;

// create a new node class to inherit from the rclcpp::Node class
class ArkermanDriveControllerNode : public rclcpp::Node
{
public:
    using StringMsg = std_msgs::msg::String;
    using TwistMsg = geometry_msgs::msg::Twist;
    using Float64MultiArrayMsg = std_msgs::msg::Float64MultiArray;

    // create the node constructor
    ArkermanDriveControllerNode() : Node("arkerman_drive_controller_node")
    {
        _wheel_vel_publisher = this->create_publisher<Float64MultiArrayMsg>("velocity_controller/commands", 10);
        _wheel_pos_publisher = this->create_publisher<Float64MultiArrayMsg>("position_controller/commands", 10);
        _cmd_subscriber = this->create_subscription<Float64MultiArrayMsg>("drive_cmd", 10, std::bind(&ArkermanDriveControllerNode::driveCallbackFunction, this, _1));
    }

private:
    // std::shared_ptr<rclcpp::Subscription<TwistMsg>> _cmd_subscriber;
    rclcpp::Subscription<Float64MultiArrayMsg>::SharedPtr _cmd_subscriber;
    rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr _wheel_vel_publisher;
    rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr _wheel_pos_publisher;

    double V_robot=0.0, W_robot=0.0, R=0.0;
    double steer_angle_rad = 0, steer_angle_deg;

    // robot parameter
    double max_steer_angle_deg = 50.0;
    double wheelRadius = 0.05;
    double D1 = 0.3, L1 = 0.238, L = 0.342, offset=0.031;

    double deg2rad(float degree){
        double pi = 3.14159265359;
        return (degree * (pi/180));
    }

private:
    void driveCallbackFunction(const Float64MultiArrayMsg::SharedPtr _cmd){

        Float64MultiArrayMsg wheel_vel_command;
        Float64MultiArrayMsg wheel_pos_command;

        double wl1=0.0, wr1=0.0, wl=0.0, wr=0.0;
        double theta_l1=0.0, theta_r1=0.0;
        

        V_robot = _cmd->data[0];
        steer_angle_deg = _cmd->data[1];
        if(steer_angle_deg>=max_steer_angle_deg) steer_angle_deg = max_steer_angle_deg;
        steer_angle_rad = this->deg2rad(steer_angle_deg);
        // RCLCPP_INFO(this->get_logger(), "v_robot: %0.3f, steer_angle: %0.3f", V_robot, steer_angle_rad);


        if(steer_angle_rad!=0){
            R = D1/tan(steer_angle_rad);

            W_robot = V_robot/R;

            theta_l1 = atan(D1/(R-(L1/2.0)));
            theta_r1 = atan(D1/(R+(L1/2.0)));

            wl1 = (W_robot*((D1/sin(theta_l1))-offset))/wheelRadius;
            wr1 = (W_robot*((D1/sin(theta_r1))+offset))/wheelRadius;

            wl = (W_robot*(R-(L/2.0)))/wheelRadius;
            wr = (W_robot*(R+(L/2.0)))/wheelRadius;
        }
        else {
            theta_l1 = 0;
            theta_r1 = 0;

            wl1 = V_robot/wheelRadius;
            wr1 = V_robot/wheelRadius;

            wl = V_robot/wheelRadius;
            wr = V_robot/wheelRadius;
        }  

        // RCLCPP_INFO(this->get_logger(), "t_l1: %0.3f, t_r1: %0.3f \nt_l2: %0.3f, t_r2: %0.3f", theta_l1, theta_r1, theta_l2, theta_r2);

        wheel_vel_command.data = {wl1, wr1, wl, wr};
        wheel_pos_command.data = {theta_l1, theta_r1};

        _wheel_pos_publisher->publish(wheel_pos_command);
        _wheel_vel_publisher->publish(wheel_vel_command);
    }
};





int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Start the TalkerNode
    rclcpp::spin(std::make_shared<ArkermanDriveControllerNode>());

    // Shutdown the node when finished
    rclcpp::shutdown();

    return 0;
}