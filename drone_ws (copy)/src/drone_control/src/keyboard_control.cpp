#include "drone_control/keyboard_control.h"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Constructor
KeyboardControl::KeyboardControl()
{

    // Initialize publishers and service clients
    vel_pub_ = nh_.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);
    takeoff_client_ = nh_.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client_ = nh_.serviceClient<airsim_ros::Land>("/airsim_node/drone_1/land");

    // Initialize subscribers
    imu_sub_ = nh_.subscribe("/airsim_node/drone_1/imu/imu", 1, &KeyboardControl::imuCallback, this);
    gps_sub_ = nh_.subscribe("/airsim_node/drone_1/gps", 1, &KeyboardControl::gpsCallback, this);

    // Initialize parameters
    linear_vel_step_ = 0.5;  // Linear velocity increment
    angular_vel_step_ = 0.5; // Angular velocity increment
    max_linear_vel_ = 2.0;   // Maximum linear velocity
    max_angular_vel_ = 0.8;  // Maximum angular velocity
    control_rate_ = 20.0;    // 20Hz control frequency

    // Initialize velocity command
    vel_cmd_.twist.linear.x = 0;
    vel_cmd_.twist.linear.y = 0;
    vel_cmd_.twist.linear.z = 0;
    vel_cmd_.twist.angular.z = 0;

    // Initialize service requests
    takeoff_srv_.request.waitOnLastTask = true;
    land_srv_.request.waitOnLastTask = true;

    // Initialize terminal
    initTerminal();
    _stop = false;
}

// IMU callback function
void KeyboardControl::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    current_imu_ = *msg;
}

// GPS callback function
void KeyboardControl::gpsCallback(const airsim_ros::GPSYaw::ConstPtr &msg)
{
    current_gps_ = *msg;
}

void KeyboardControl::resetVelCmd()
{
    vel_cmd_.twist.linear.x = 0;
    vel_cmd_.twist.linear.y = 0;
    vel_cmd_.twist.linear.z = 0;
    vel_cmd_.twist.angular.z = 0;
}

// Process keyboard input
void KeyboardControl::processKey(char key)
{
    switch (key)
    {
    case 'w': // Forward
        vel_cmd_.twist.linear.x = max_linear_vel_;
        ROS_INFO("Forward");
        break;
    case 's': // Backward
        vel_cmd_.twist.linear.x = -max_linear_vel_;
        ROS_INFO("Backward");
        break;
    case 'a': // Left
        vel_cmd_.twist.linear.y = -max_linear_vel_;
        ROS_INFO("Left");
        break;
    case 'd': // Right
        vel_cmd_.twist.linear.y = max_linear_vel_;
        ROS_INFO("Right");
        break;
    case 'q': // Turn left
        vel_cmd_.twist.angular.z = -max_angular_vel_;
        ROS_INFO("Turn Left");
        break;
    case 'e': // Turn right
        vel_cmd_.twist.angular.z = max_angular_vel_;
        ROS_INFO("Turn Right");
        break;
    case 'r': // Up
        vel_cmd_.twist.linear.z = -max_linear_vel_;
        ROS_INFO("Up");
        break;
    case 'f': // Down
        vel_cmd_.twist.linear.z = max_linear_vel_;
        ROS_INFO("Down");
        break;
    case ' ': // Stop
        resetVelCmd();
        ROS_INFO("Stop");
        _stop = true;
        break;
    case 't': // Takeoff
        if (takeoff_client_.call(takeoff_srv_))
        {
            ROS_INFO("Takeoff command sent");
        }
        else
        {
            ROS_ERROR("Failed to send takeoff command");
        }
        break;
    case 'g': // Land
        if (land_client_.call(land_srv_))
        {
            ROS_INFO("Land command sent");
        }
        else
        {
            ROS_ERROR("Failed to send land command");
        }
        break;
    case 'h': // Help
        showInstructions();
        break;
    }

    ROS_INFO("Velocity command: Linear(%.2f, %.2f, %.2f) m/s, Angular %.2f rad/s",
             vel_cmd_.twist.linear.x, vel_cmd_.twist.linear.y, vel_cmd_.twist.linear.z,
             vel_cmd_.twist.angular.z);
}

// Publish velocity command
void KeyboardControl::publishVelCmd()
{
    vel_pub_.publish(vel_cmd_);
}

// Show operation instructions
void KeyboardControl::showInstructions()
{
    std::cout << "\nKeyboard Control Instructions:" << std::endl;
    std::cout << "w/s: Forward/Backward" << std::endl;
    std::cout << "a/d: Left/Right" << std::endl;
    std::cout << "q/e: Turn Left/Right" << std::endl;
    std::cout << "r/f: Up/Down" << std::endl;
    std::cout << "Space: Stop" << std::endl;
    std::cout << "t: Takeoff" << std::endl;
    std::cout << "g: Land" << std::endl;
    std::cout << "h: Show Help" << std::endl;
    std::cout << "Ctrl+C: Exit\n"
              << std::endl;
}

// Run function
void KeyboardControl::run()
{
    showInstructions();
    ROS_INFO("Node started, waiting for input...");

    ros::Rate rate(control_rate_);

    while (ros::ok() && !_stop)
    {

        char key = readKey();
        if (key != -1)
        {
            processKey(key);
            publishVelCmd();
        }
        else
        {
            resetVelCmd();
            publishVelCmd();
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Node exiting normally");
}

// Destructor
KeyboardControl::~KeyboardControl()
{
    restoreTerminal();
}

// Initialize terminal
void KeyboardControl::initTerminal()
{
    tcgetattr(0, &orig_termios_);
    struct termios new_termios = orig_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &new_termios);
}

// Restore terminal settings
void KeyboardControl::restoreTerminal()
{
    tcsetattr(0, TCSANOW, &orig_termios_);
}

// Read keyboard input
char KeyboardControl::readKey()
{
    char key;
    struct timeval tv;
    fd_set readfds;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 0.1 seconds
    FD_ZERO(&readfds);
    FD_SET(0, &readfds);
    if (select(1, &readfds, NULL, NULL, &tv) > 0)
    {
        if (read(0, &key, 1) < 0)
        {
            return -1;
        }
        return key;
    }
    return -1;
}

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control");
    KeyboardControl controller;
    controller.run();
    return 0;
}