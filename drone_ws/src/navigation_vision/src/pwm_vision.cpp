#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <airsim_ros/VelCmd.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
class DroneController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber position_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber avoid_car_sub_;
    ros::Publisher cmd_vel_pub_;

    // PID parameters loaded from config
    double kp_yaw_;
    double ki_yaw_;
    double kd_yaw_;
    double kp_z_;
    double ki_z_;
    double kd_z_;
    double k_y_;

    double y_output_;
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

    ros::Timer reset_timer_;
    ros::Time last_avoid_time_;
    double reset_timeout_; // seconds

    // Add PID parameters for y control
    double kp_y_;
    double ki_y_;
    double kd_y_;
    double y_integral_;
    double prev_y_error_;

public:
    DroneController() : nh_("~"), y_output_(0.0)
    {
        // Load PID parameters from config
        nh_.param<double>("pid/yaw/kp", kp_yaw_, 0.003);
        nh_.param<double>("pid/yaw/ki", ki_yaw_, 0.0001);
        nh_.param<double>("pid/yaw/kd", kd_yaw_, 0.001);
        nh_.param<double>("pid/z/kp", kp_z_, 0.004);
        nh_.param<double>("pid/z/ki", ki_z_, 0.0001);
        nh_.param<double>("pid/z/kd", kd_z_, 0.001);
        // Load y PID parameters from config
        nh_.param<double>("pid/y/kp", kp_y_, 0.5);
        nh_.param<double>("pid/y/ki", ki_y_, 0.01);
        nh_.param<double>("pid/y/kd", kd_y_, 0.1);

        // Load control limits from config
        nh_.param<double>("control/max_yaw_rate", MAX_YAW_RATE, 10.0);
        nh_.param<double>("control/max_z_speed", MAX_Z_SPEED, 20.0);
        nh_.param<double>("control/forward_speed", FORWARD_SPEED, 0.0);

        // Add timeout parameter (default 0.5 seconds)
        nh_.param<double>("control/reset_timeout", reset_timeout_, 0.5);

        // Initialize timer to check and reset y_output_
        reset_timer_ = nh_.createTimer(ros::Duration(0.1), &DroneController::resetCheck, this);
        last_avoid_time_ = ros::Time::now();

        // Get topics from config
        std::string position_topic, cmd_vel_topic, pose_topic, avoid_car_topic;
        nh_.param<std::string>("topics/position", position_topic, "/airsim_node/drone_1/front_right/position");
        nh_.param<std::string>("topics/cmd_vel", cmd_vel_topic, "/airsim_node/drone_1/vel_cmd_body_frame");
        nh_.param<std::string>("topics/drone_pose", pose_topic, "/airsim_node/drone_1/drone_pose");
        nh_.param<std::string>("topics/avoid_car", avoid_car_topic, "/airsim_node/drone_1/front_right/avoid_car");
        // Initialize subscribers and publishers
        position_sub_ = nh_.subscribe(position_topic, 1, &DroneController::positionCallback, this);
        cmd_vel_pub_ = nh_.advertise<airsim_ros::VelCmd>(cmd_vel_topic, 1);
        pose_sub_ = nh_.subscribe(pose_topic, 1, &DroneController::poseCallback, this);
        avoid_car_sub_ = nh_.subscribe(avoid_car_topic, 1, &DroneController::avoidCarCallback, this);
        // Initialize error tracking
        yaw_integral_ = 0;
        z_integral_ = 0;
        prev_yaw_error_ = 0;
        prev_z_error_ = 0;
        prev_time_ = ros::Time::now().toSec();

        // Initialize y error tracking
        y_integral_ = 0;
        prev_y_error_ = 0;

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
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.linear.y = y_output_;
        cmd_vel.twist.linear.z = 0;
        cmd_vel.twist.angular.x = 0.0;
        cmd_vel.twist.angular.y = 0.;
        cmd_vel.twist.angular.z = 0;

        ROS_INFO("Publishing velocity command: x=%f, y=%f, z=%f, yaw=%f", cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z, cmd_vel.twist.angular.z);
        cmd_vel_pub_.publish(cmd_vel);

        // Update previous values
        prev_yaw_error_ = yaw_error;
        prev_z_error_ = z_error;
        prev_time_ = current_time;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // ROS_INFO("Received drone pose: x=%f, y=%f, z=%f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        // resetIntegral();
    }

    void avoidCarCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() > 0)
        {
            double current_time = ros::Time::now().toSec();
            double dt = current_time - prev_time_;
            if (dt <= 0)
                return;

            // PID control for y (based on car avoidance error)
            double y_error = msg->data[0];
            y_integral_ += y_error * dt;
            double y_derivative = (y_error - prev_y_error_) / dt;

            y_output_ = kp_y_ * y_error +
                        ki_y_ * y_integral_ +
                        kd_y_ * y_derivative;

            // Limit y_output to [-1, 1]
            y_output_ = std::max(-1.0, std::min(y_output_, 1.0));

            last_avoid_time_ = ros::Time::now();
            prev_y_error_ = y_error;
            prev_time_ = current_time;

            // ROS_INFO("y_output: %f", y_output_);
        }
    }

    void resetCheck(const ros::TimerEvent &)
    {
        if ((ros::Time::now() - last_avoid_time_).toSec() > reset_timeout_)
        {
            if (y_output_ != 0.0)
            {
                y_output_ = 0.0;
                y_integral_ = 0.0;                     // Reset integral term
                prev_y_error_ = 0.0;                   // Reset previous error
                prev_time_ = ros::Time::now().toSec(); // Reset time
                ROS_INFO("Reset y_output, integral, and time variables to 0");
            }
        }
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
