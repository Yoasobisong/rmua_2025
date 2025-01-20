#include <ros/ros.h>
#include <navigation_vision/Box.h>
#include <airsim_ros/VelCmd.h>

class PWMController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Publisher velocity_pub_;
    float init_x = 480.;
    float init_y = 360.;
    float k_angular_z = 0.002;
    float k_linear_z = 0.003;
    airsim_ros::VelCmd vel_cmd_;
    // Callback function for target position
    void targetCallback(const navigation_vision::Box::ConstPtr &msg)
    {
        // Get target position from message
        float target_x = msg->box[0];
        float target_y = msg->box[1];
        if (msg->box[2] == -1)
        {
            vel_cmd_.twist.linear.z = 0;
            vel_cmd_.twist.angular.z = 0;
            vel_cmd_.twist.linear.x = 1;
            vel_cmd_.twist.linear.y = 0;
        }
        else
        {
            vel_cmd_.twist.linear.x = 3;
            vel_cmd_.twist.linear.y = 0;
            vel_cmd_.twist.linear.z = k_linear_z * (target_y - init_y);
            vel_cmd_.twist.angular.z = k_angular_z * (target_x - init_x);
        }
        velocity_pub_.publish(vel_cmd_);
        ROS_INFO("target_x: %.2f, target_y: %.2f", target_x, target_y);
        ROS_INFO("Published velocity command: linear_x=%.2f, linear_y=%.2f, linear_z=%.2f, angular_z=%.2f", vel_cmd_.twist.linear.x, vel_cmd_.twist.linear.y, vel_cmd_.twist.linear.z, vel_cmd_.twist.angular.z);
    }

public:
    PWMController()
    {
        // Initialize subscriber
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
