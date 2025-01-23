#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <airsim_ros/VelCmd.h>
#include <nav_msgs/Odometry.h>

class DroneController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber position_sub_;
    ros::Publisher cmd_vel_pub_;

    // PID parameters loaded from config
    double kp_yaw_;
    double ki_yaw_;
    double kd_yaw_;
    double kp_z_;
    double ki_z_;
    double kd_z_;

    // Control limits loaded from config
    double MAX_YAW_RATE;
    double MAX_Z_SPEED;
    double FORWARD_SPEED;

    // Error tracking
    double yaw_integral_;
    double z_integral_;
    double prev_yaw_error_;
    double prev_z_error_;
    double prev_time_;

    // Image center (assuming 640x480 resolution)
    const int IMAGE_CENTER_X = 480;
    const int IMAGE_CENTER_Y = 360;

public:
    DroneController() : nh_("~")
    {
        // Load PID parameters from config
        nh_.param<double>("pid/yaw/kp", kp_yaw_, 0.003);
        nh_.param<double>("pid/yaw/ki", ki_yaw_, 0.0001);
        nh_.param<double>("pid/yaw/kd", kd_yaw_, 0.001);
        nh_.param<double>("pid/z/kp", kp_z_, 0.0);
        nh_.param<double>("pid/z/ki", ki_z_, 0.0);
        nh_.param<double>("pid/z/kd", kd_z_, 0.0);

        // Load control limits from config
        nh_.param<double>("control/max_yaw_rate", MAX_YAW_RATE, 10.0);
        nh_.param<double>("control/max_z_speed", MAX_Z_SPEED, 20.0);
        nh_.param<double>("control/forward_speed", FORWARD_SPEED, 0.0);

        // Get topics from config
        std::string position_topic, cmd_vel_topic;
        nh_.param<std::string>("topics/position", position_topic, "/airsim_node/drone_1/front_right/position");
        nh_.param<std::string>("topics/cmd_vel", cmd_vel_topic, "/airsim_node/drone_1/vel_cmd_body_frame");

        // Initialize subscribers and publishers
        position_sub_ = nh_.subscribe(position_topic, 1, &DroneController::positionCallback, this);
        cmd_vel_pub_ = nh_.advertise<airsim_ros::VelCmd>(cmd_vel_topic, 1);

        // Initialize error tracking
        yaw_integral_ = 0;
        z_integral_ = 0;
        prev_yaw_error_ = 0;
        prev_z_error_ = 0;
        prev_time_ = ros::Time::now().toSec();

        ROS_INFO("Drone controller initialized with parameters from config");
        // print parameters
        ROS_INFO("kp_yaw_: %f", kp_yaw_);
        ROS_INFO("ki_yaw_: %f", ki_yaw_);
        ROS_INFO("kd_yaw_: %f", kd_yaw_);
        ROS_INFO("kp_z_: %f", kp_z_);
        ROS_INFO("ki_z_: %f", ki_z_);
        ROS_INFO("kd_z_: %f", kd_z_);
    }

    void positionCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() < 3)
            return;

        double current_time = ros::Time::now().toSec();
        double dt = current_time - prev_time_;
        if (dt <= 0)
            return;

        // Calculate errors (target position - image center)
        double x_error = msg->data[0] - IMAGE_CENTER_X;
        double y_error = msg->data[1] - IMAGE_CENTER_Y;

        // PID control for yaw (based on x error)
        double yaw_error = x_error;
        yaw_integral_ += yaw_error * dt;
        double yaw_derivative = (yaw_error - prev_yaw_error_) / dt;
        double yaw_output = kp_yaw_ * yaw_error +
                            ki_yaw_ * yaw_integral_ +
                            kd_yaw_ * yaw_derivative;

        // PID control for z (based on y error)
        double z_error = y_error;
        z_integral_ += z_error * dt;
        double z_derivative = (z_error - prev_z_error_) / dt;
        double z_output = kp_z_ * z_error +
                          ki_z_ * z_integral_ +
                          kd_z_ * z_derivative;

        // Apply limits
        yaw_output = std::max(-MAX_YAW_RATE, std::min(yaw_output, MAX_YAW_RATE));
        z_output = std::max(-MAX_Z_SPEED, std::min(z_output, MAX_Z_SPEED));

        // Create and publish velocity command
        airsim_ros::VelCmd cmd_vel;
        cmd_vel.twist.linear.x = FORWARD_SPEED;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.linear.z = z_output;
        cmd_vel.twist.angular.x = 0.0;
        cmd_vel.twist.angular.y = 0.0;
        cmd_vel.twist.angular.z = yaw_output;

        ROS_INFO("Publishing velocity command: x=%f, y=%f, z=%f, yaw=%f", cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z, cmd_vel.twist.angular.z);
        cmd_vel_pub_.publish(cmd_vel);

        // Update previous values
        prev_yaw_error_ = yaw_error;
        prev_z_error_ = z_error;
        prev_time_ = current_time;
    }

    // Reset integral terms if needed
    void resetIntegral()
    {
        yaw_integral_ = 0;
        z_integral_ = 0;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm_vision");
    DroneController controller;
    ros::spin();
    return 0;
}
