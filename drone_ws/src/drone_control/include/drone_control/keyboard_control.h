#ifndef KEYBOARD_CONTROL_H
#define KEYBOARD_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <airsim_ros/VelCmd.h>
#include <airsim_ros/Takeoff.h>
#include <airsim_ros/Land.h>
#include <airsim_ros/GPSYaw.h>
#include <sensor_msgs/Imu.h>
#include <termios.h>
#include <unistd.h>

class KeyboardControl {
public:
    KeyboardControl();
    ~KeyboardControl();
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    
    airsim_ros::VelCmd vel_cmd_;
    airsim_ros::Takeoff takeoff_srv_;
    airsim_ros::Land land_srv_;
    
    sensor_msgs::Imu current_imu_;
    airsim_ros::GPSYaw current_gps_;
    
    bool _stop;
    double linear_vel_step_;
    double angular_vel_step_;
    double max_linear_vel_;
    double max_angular_vel_;
    double control_rate_;
    
    struct termios orig_termios_;
    
    void initTerminal();
    void restoreTerminal();
    char readKey();
    void processKey(char key);
    void publishVelCmd();
    void showInstructions();
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gpsCallback(const airsim_ros::GPSYaw::ConstPtr& msg);
    void resetVelCmd();
};

#endif // KEYBOARD_CONTROL_H 