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
        auto subscriber = this->create_subscription<std_msgs::msg::String>(
            "/rosbot_base_controller/odom", 10,
            [&](const std_msgs::msg::String& msg){
                std::cout << "I heard: x " << msg.data << std::endl;
            }
            );

    }


private:

    // You can use this streams to output
    // the coordinates of your computed
    // poses
    std::ofstream& odom_out_;
    std::ofstream& imu_out_;


};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt");
    std::ofstream imu_out("imu.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}