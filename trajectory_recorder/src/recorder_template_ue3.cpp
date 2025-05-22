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
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_broadcaster/imu", 
            10, 
            std::bind(&TrajectoryRecorder::imu_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "rosbot_base_controller/odom", 
            10, 
            std::bind(&TrajectoryRecorder::odom_callback, this, _1));

        global_x_ = 0;
        global_y_ = 0;
        global_z_ = 0;
        last_time_ = 0;
        first_ = true;
        first_odom_ = true;
    }


private:

    void imu_callback(const sensor_msgs::msg::Imu& msg) 
    {
        
    }

    void odom_callback(const nav_msgs::msg::Odometry& msg) 
    {
        geometry_msgs::msg::PoseWithCovariance pose = msg.pose;
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double z = pose.pose.position.z;

        if(first_odom_)
        {
            first_odom_ = false;
            odom_offset_x_ = pose.pose.position.x;
            odom_offset_y_ = pose.pose.position.y;
            odom_offset_z_ = pose.pose.position.z;
        }
        else
        {
            // to make sure that the position is in the scope and the original point is 0,0
            x = x - odom_offset_x_;
            y = y - odom_offset_y_;
            z = z - odom_offset_z_;
        }

        if(odom_out_.good())
        {
            odom_out_ << x << " " << y << " " << z << std::endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::ofstream& odom_out_;
    std::ofstream& imu_out_;

    double global_x_;
    double global_y_;
    double global_z_;

    double odom_offset_x_ = 0.0;
    double odom_offset_y_ = 0.0;
    double odom_offset_z_ = 0.0;

    double last_vel_x_ = 0.0;
    double last_vel_y_ = 0.0;
    double last_vel_z_ = 0.0;

    bool first_;
    bool first_odom_;

    double last_time_ = 0;
};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt");
    std::ofstream imu_out("imu.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}