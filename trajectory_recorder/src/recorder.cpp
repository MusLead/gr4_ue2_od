#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class TrajectoryRecorder : public rclcpp::Node
{
public:
    TrajectoryRecorder(std::ofstream& odom_out, std::ofstream& imu_out) : 
        Node("trajectory_recorder"), odom_out_(odom_out), imu_out_(imu_out)
    {
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "rosbot_base_controller/odom", 10,
            [&](const nav_msgs::msg::Odometry& msg){
                std::cout << "[Odom] x " << msg.pose.pose.position.x 
                    << ", y "<< msg.pose.pose.position.y 
                    << std::endl;
                odom_out_ << msg.pose.pose.position.x << " "
                          << msg.pose.pose.position.y  << std::endl;
                odom_out_.flush();
            }
        );

        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_broadcaster/imu", 10,
            [&](const sensor_msgs::msg::Imu& msg){
                std::cout << "[IMU] time: " << msg.header.stamp.nanosec << std::endl;
                imu_out_ << msg.orientation.x << " "
                         << msg.orientation.y << " "
                         << msg.orientation.z << " "
                         << msg.orientation.w << " "
                         << msg.angular_velocity.x << " "
                         << msg.angular_velocity.y << " "
                         << msg.angular_velocity.z << " "
                         << msg.linear_acceleration.x << " "
                         << msg.linear_acceleration.y << " "
                         << msg.linear_acceleration.z << std::endl;
                imu_out_.flush();
            }
        );
  
    }


private:

    // You can use this streams to output
    // the coordinates of your computed
    // poses
    std::ofstream& odom_out_;
    std::ofstream& imu_out_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;

};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt", std::ios::out | std::ios::trunc);
    std::ofstream imu_out("imu.txt", std::ios::out | std::ios::trunc);


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}