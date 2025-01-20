#include <ros/ros.h>
#include <navigation_vision/Box.h>
#include <airsim_ros/VelCmd.h>

class PWMController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Publisher velocity_pub_;
    float init_x_;
    float init_y_;

    float kp_linear_z_;
    float ki_linear_z_;
    float kd_linear_z_;

    float kp_angular_z_;
    float ki_angular_z_;
    float kd_angular_z_;

    float search_vel_x_;
    float track_vel_x_;
    float search_vel_z_;

    airsim_ros::VelCmd vel_cmd_;
    // Add error history and integral terms for both x and y
    float prev_error_x_ = 0.0;
    float prev_error_y_ = 0.0;
    float integral_x_ = 0.0;
    float integral_y_ = 0.0;

    void loadParameters()
    {
        // Load initial positions
        nh_.param<float>("pwm_controller/init_x", init_x_, 480.0);
        nh_.param<float>("pwm_controller/init_y", init_y_, 360.0);

        // Load linear z PID parameters
        nh_.param<float>("pwm_controller/linear_z/kp", kp_linear_z_, 0.020);
        nh_.param<float>("pwm_controller/linear_z/ki", ki_linear_z_, 0.0001);
        nh_.param<float>("pwm_controller/linear_z/kd", kd_linear_z_, 0.001);

        // Load angular z PID parameters
        nh_.param<float>("pwm_controller/angular_z/kp", kp_angular_z_, 0.0034);
        nh_.param<float>("pwm_controller/angular_z/ki", ki_angular_z_, 0.0001);
        nh_.param<float>("pwm_controller/angular_z/kd", kd_angular_z_, 0.001);

        // Load velocity parameters
        nh_.param<float>("pwm_controller/linear_velocity/search_x", search_vel_x_, 3.0);
        nh_.param<float>("pwm_controller/linear_velocity/track_x", track_vel_x_, 7.0);
        nh_.param<float>("pwm_controller/linear_velocity/search_z", search_vel_z_, -0.2);
    }

    void targetCallback(const navigation_vision::Box::ConstPtr &msg)
    {
        float error_x_ = msg->box[0] - init_x_;
        float error_y_ = msg->box[1] - init_y_;

        if (msg->box[2] == -1)
        {
            vel_cmd_.twist.linear.z = search_vel_z_;
            vel_cmd_.twist.angular.z = 0;
            vel_cmd_.twist.linear.x = search_vel_x_;
            vel_cmd_.twist.linear.y = 0;
        }
        else
        {
            vel_cmd_.twist.linear.x = track_vel_x_;
            vel_cmd_.twist.linear.y = 0;
            pid_control_(error_x_, error_y_);
        }
        // velocity_pub_.publish(vel_cmd_);
        ROS_INFO("target_x: %.2f, target_y: %.2f", error_x_, error_y_);
        ROS_INFO("Published velocity command: linear_x=%.2f, linear_y=%.2f, linear_z=%.2f, angular_z=%.2f",
                 vel_cmd_.twist.linear.x, vel_cmd_.twist.linear.y, vel_cmd_.twist.linear.z, vel_cmd_.twist.angular.z);
    }

    void pid_control_(float &error_x_, float &error_y_)
    {
        // Calculate derivative terms
        float derivative_x = error_x_ - prev_error_x_;
        float derivative_y = error_y_ - prev_error_y_;

        // Update integral terms
        integral_x_ += error_x_;
        integral_y_ += error_y_;

        // Calculate PID output for x (angular.z)
        vel_cmd_.twist.angular.z = kp_angular_z_ * error_x_ +
                                   ki_angular_z_ * integral_x_ +
                                   kd_angular_z_ * derivative_x;

        // Calculate PID output for y (linear.z)
        vel_cmd_.twist.linear.z = kp_linear_z_ * error_y_ +
                                  ki_linear_z_ * integral_y_ +
                                  kd_linear_z_ * derivative_y;

        // Update previous errors for next iteration
        prev_error_x_ = error_x_;
        prev_error_y_ = error_y_;
    }

public:
    PWMController()
    {
        loadParameters();
        target_sub_ = nh_.subscribe("/airsim_node/drone_1/front_right/target_position", 1,
                                    &PWMController::targetCallback, this);
        velocity_pub_ = nh_.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);
        ROS_INFO("PWM Controller initialized");
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm_ctrl");

    PWMController controller;
    controller.run();

    return 0;
}
