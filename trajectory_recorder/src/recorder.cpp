#include <memory>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;
using std::sqrt;
using std::pow;
using std::sin;
using std::cos;

class TrajectoryRecorder : public rclcpp::Node
{
public:
    TrajectoryRecorder(std::ofstream& odom_out, std::ofstream& imu_out, std::ofstream& imu_debug_out) : 
        Node("trajectory_recorder"), odom_out_(odom_out), imu_out_(imu_out), imu_debug_out_(imu_debug_out)
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

             	double t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
				if (t_vor == 0) {
                    
                    ax = msg.linear_acceleration.x;
					ay = msg.linear_acceleration.y;
					az = msg.linear_acceleration.z;
                    omega = msg.angular_velocity.z;

				} else if(ax != msg.linear_acceleration.x || 
                    ay != msg.linear_acceleration.y || 
                    az != msg.linear_acceleration.z ||
                    omega != msg.angular_velocity.z) {

                    log_imu_debug(msg);
					
                    ax = msg.linear_acceleration.x;
					ay = msg.linear_acceleration.y;
					az = msg.linear_acceleration.z;	
					omega = msg.angular_velocity.z;

					double delta_t = t_now - t_vor;
					double v = sqrt(pow(ax * delta_t, 2.0) + pow(ay * delta_t, 2.0) + pow(az * delta_t, 2.0));
					double xn = x_vor + (v * delta_t * sin(rotateAngular_vor));
					double yn = y_vor + (v * delta_t * cos(rotateAngular_vor));
					double rotateAngular = rotateAngular_vor + (omega * pow(delta_t, 2.0));
	
					std::cout <<"[IMU] x " << xn << ", y " << yn << std::endl;
					imu_out << xn << " " << yn << std::endl;
	                imu_out_.flush();
	
					x_vor = xn;
					y_vor = yn;
					rotateAngular_vor = rotateAngular;
				}
                t_vor = t_now;
            }
        );
  
    }


private:
    void log_imu_debug(const sensor_msgs::msg::Imu& msg)
    {
        imu_debug_out_ << "[IMU DEBUG] Orientation: ("
               << msg.orientation.x << ", "
               << msg.orientation.y << ", "
               << msg.orientation.z << ", "
               << msg.orientation.w << "), "
               << "Angular Velocity: ("
               << msg.angular_velocity.x << ", "
               << msg.angular_velocity.y << ", "
               << msg.angular_velocity.z << "), "
               << "Linear Acceleration: ("
               << msg.linear_acceleration.x << ", "
               << msg.linear_acceleration.y << ", "
               << msg.linear_acceleration.z << ")"
               << std::endl;

        imu_debug_out_.flush();
    }

    // You can use this streams to output
    // the coordinates of your computed
    // poses
    std::ofstream& odom_out_;
    std::ofstream& imu_out_;
    std::ofstream& imu_debug_out_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
	
    bool isMoving = false;
    double x_odom = 0, y_odom = 0;
    double ax, ay, az, omega;
	double t_vor = 0;
	double x_vor = 0, y_vor = 0;
	double rotateAngular_vor = 0;
};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt", std::ios::out | std::ios::trunc);
    std::ofstream imu_out("imu.txt", std::ios::out | std::ios::trunc);
    std::ofstream imu_debug_out_("imu_debug.txt", std::ios::out | std::ios::trunc);


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out, imu_debug_out_));
    rclcpp::shutdown();
}