#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

class GPSFilter
{
private:
    ros::NodeHandle nh_;
    ros::Publisher filtered_pub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;

    // 最新的IMU姿态数据
    geometry_msgs::Quaternion latest_imu_orientation_;
    bool has_imu_data_;

    // 上一次的位置数据
    geometry_msgs::Point last_position_;
    bool has_last_position_;

    // 滤波参数
    static constexpr double ALPHA = 0.3; // 低通滤波系数 (0-1)，越小滤波越强

public:
    GPSFilter() : has_imu_data_(false), has_last_position_(false)
    {
        ROS_INFO("Initializing GPS Filter node with low-pass filter...");
        ROS_INFO("Alpha: %f", ALPHA);

        // 设置ROS话题
        filtered_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/airsim_node/drone_1/filtered_gps", 10);
        gps_sub_ = nh_.subscribe("/airsim_node/drone_1/gps", 10, &GPSFilter::gpsCallback, this);
        imu_sub_ = nh_.subscribe("/airsim_node/drone_1/imu/imu", 10, &GPSFilter::imuCallback, this);

        ROS_INFO("GPS Filter initialized");
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        latest_imu_orientation_ = msg->orientation;
        has_imu_data_ = true;
    }

    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!has_imu_data_)
        {
            ROS_WARN_THROTTLE(1.0, "Waiting for IMU data...");
            return;
        }

        geometry_msgs::PoseStamped filtered_msg;
        filtered_msg.header = msg->header;

        if (has_last_position_)
        {
            // 应用低通滤波器
            filtered_msg.pose.position.x = ALPHA * msg->pose.position.x + (1 - ALPHA) * last_position_.x;
            filtered_msg.pose.position.y = ALPHA * msg->pose.position.y + (1 - ALPHA) * last_position_.y;
            filtered_msg.pose.position.z = ALPHA * msg->pose.position.z + (1 - ALPHA) * last_position_.z;
        }
        else
        {
            // 第一次收到数据，直接使用
            filtered_msg.pose.position = msg->pose.position;
            has_last_position_ = true;
        }

        // 使用IMU的姿态
        filtered_msg.pose.orientation = latest_imu_orientation_;

        // 保存当前位置用于下次滤波
        last_position_ = filtered_msg.pose.position;

        filtered_pub_.publish(filtered_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_filter");
    GPSFilter filter;
    ros::spin();
    return 0;
}