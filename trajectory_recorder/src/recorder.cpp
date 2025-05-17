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
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "rosbot_base_controller/odom", 10,
            [&](const nav_msgs::msg::Odometry& msg){
                std::cout << "I heard: x " << msg.pose.pose.position.x << std::endl;
            }
            );
    }


private:

    // You can use this streams to output
    // the coordinates of your computed
    // poses
    std::ofstream& odom_out_;
    std::ofstream& imu_out_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt");
    std::ofstream imu_out("imu.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}