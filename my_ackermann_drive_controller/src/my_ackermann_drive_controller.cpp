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
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"



using namespace std::placeholders;
using namespace std::chrono_literals;

// create a new node class to inherit from the rclcpp::Node class
class ArkermanDriveControllerNode : public rclcpp::Node
{
public:
    using StringMsg = std_msgs::msg::String;
    using TwistMsg = geometry_msgs::msg::Twist;
    using Float64MultiArrayMsg = std_msgs::msg::Float64MultiArray;
    using AckermannDriveMsg = ackermann_msgs::msg::AckermannDrive;
    using JointStateMsg = sensor_msgs::msg::JointState;
    using OdometryMsg = nav_msgs::msg::Odometry;

    using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

    // create the node constructor
    ArkermanDriveControllerNode() : Node("arkerman_drive_controller_node")
    {
        _wheel_vel_publisher = this->create_publisher<Float64MultiArrayMsg>("velocity_controller/commands", 10);
        _wheel_pos_publisher = this->create_publisher<Float64MultiArrayMsg>("position_controller/commands", 10);
        _cmd_subscriber = this->create_subscription<AckermannDriveMsg>("cmd_ackermann", 10, std::bind(&ArkermanDriveControllerNode::driveCallbackFunction, this, _1));

        _timer = this->create_wall_timer(25ms, std::bind(&ArkermanDriveControllerNode::computeAndPublishOdom, this));
        _joint_state_subscriber = this->create_subscription<JointStateMsg>("/joint_states", 10, std::bind(&ArkermanDriveControllerNode::readJointState, this, _1));

        // Initialize the transform broadcaster
        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    // std::shared_ptr<rclcpp::Subscription<TwistMsg>> _cmd_subscriber;
    
    rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr _wheel_vel_publisher;
    rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr _wheel_pos_publisher;
    rclcpp::Subscription<AckermannDriveMsg>::SharedPtr _cmd_subscriber;
    rclcpp::Subscription<JointStateMsg>::SharedPtr _joint_state_subscriber;

    rclcpp::TimerBase::SharedPtr _timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    double V_robot=0.0, W_robot=0.0, R=0.0;
    double steer_angle_rad = 0;

    // robot parameter
    double max_steer_angle_rad = 1.047196; //rads => 60 deg
    double wheelRadius = 0.05;
    double D1 = 0.3, L1 = 0.238, L = 0.342, offset=0.031;

    // double deg2rad(float degree){
    //     double pi = 3.14159265359;
    //     return (degree * (pi/180));
    // }


    // robot wheel state
    double thR, thL; // wheel position reading

    double thetaR, thetaL;
    double prevThetaR, prevThetaL;

    // robot initialize pose
    double prevPoseX, prevPoseY, prevPoseTheta;
    double poseX = 0.0, poseY = 0.0, poseTheta = 0.0;

private:
    void driveCallbackFunction(const AckermannDriveMsg::SharedPtr _cmd){

        Float64MultiArrayMsg wheel_vel_command;
        Float64MultiArrayMsg wheel_pos_command;

        double wl1=0.0, wr1=0.0, wl=0.0, wr=0.0;
        double theta_l1=0.0, theta_r1=0.0;
        

        V_robot = _cmd->speed;
        steer_angle_rad = _cmd->steering_angle;
        if(steer_angle_rad>=max_steer_angle_rad) steer_angle_rad = max_steer_angle_rad;
        
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



    void readJointState(const JointStateMsg::SharedPtr _joint_state){
      thL = _joint_state->position[4];
      thR = _joint_state->position[5];
    }

    void timer_callback(){
      auto message = std_msgs::msg::String();
      message.data = "sensor-readings: " + std::to_string(thL) + ", " + std::to_string(thR);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    void computeAndPublishOdom() {
      thetaR = thR;
      thetaL = thL;

      poseX = prevPoseX + ( (wheelRadius/2)*((thetaR-prevThetaR)+(thetaL-prevThetaL))*std::cos(prevPoseTheta) );
      poseY = prevPoseY + ( (wheelRadius/2)*((thetaR-prevThetaR)+(thetaL-prevThetaL))*std::sin(prevPoseTheta) );
      poseTheta = prevPoseTheta + (wheelRadius/L)*((thetaR-prevThetaR)-(thetaL-prevThetaL));


      ////////////////////////////////////////////////////////////////////////////
      TransformStampedMsg t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";

      t.transform.translation.x = poseX;
      t.transform.translation.y = poseY;
      t.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, poseTheta); // convert rpy to quaternion
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      _tf_broadcaster->sendTransform(t);
      ////////////////////////////////////////////////////////////////////////////


      // auto message = std_msgs::msg::String();
      // message.data = "poseX:" + std::to_string(poseX) + ", poseY:" + std::to_string(poseY) + ", poseTheta:" + std::to_string(poseTheta);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      prevThetaL = thetaL;  prevThetaR = thetaR;
      prevPoseX = poseX; prevPoseY = poseY; prevPoseTheta = poseTheta;
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