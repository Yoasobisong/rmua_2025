#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

class TFBroadcaster
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber drone_pose_sub_;
    ros::Subscriber end_goal_sub_;
    ros::Subscriber lidar_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer map_timer_;

    tf::Transform current_drone_transform_;
    bool has_drone_pose_;
    ros::Time last_drone_update_;

    bool isValidPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (std::isnan(msg->pose.position.x) || std::isnan(msg->pose.position.y) || std::isnan(msg->pose.position.z))
        {
            ROS_WARN_THROTTLE(1.0, "Invalid position data detected (NaN)");
            return false;
        }

        if (std::isnan(msg->pose.orientation.x) || std::isnan(msg->pose.orientation.y) ||
            std::isnan(msg->pose.orientation.z) || std::isnan(msg->pose.orientation.w))
        {
            ROS_WARN_THROTTLE(1.0, "Invalid orientation data detected (NaN)");
            return false;
        }

        return true;
    }

    void mapTimerCallback(const ros::TimerEvent &)
    {
        // 发布静态的map到drone_init的转换
        tf::Transform map_to_init;
        map_to_init.setIdentity(); // 设置为单位变换
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(map_to_init, ros::Time::now(),
                                 "map", "drone_init"));
    }

public:
    TFBroadcaster() : has_drone_pose_(false)
    {
        ROS_INFO("Initializing TF Broadcaster node...");

        // 设置定时器，每100ms发布一次map到drone_init的转换
        map_timer_ = nh_.createTimer(ros::Duration(0.1),
                                     &TFBroadcaster::mapTimerCallback, this);

        drone_pose_sub_ = nh_.subscribe("/airsim_node/drone_1/drone_pose", 1,
                                        &TFBroadcaster::dronePoseCallback, this);
        end_goal_sub_ = nh_.subscribe("/airsim_node/drone_1/drone_end", 1,
                                      &TFBroadcaster::endGoalCallback, this);
        lidar_sub_ = nh_.subscribe("/airsim_node/drone_1/lidar", 1,
                                   &TFBroadcaster::lidarCallback, this);

        ROS_INFO("TF Broadcaster initialized");
    }

    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!isValidPose(msg))
        {
            return;
        }

        // 更新当前无人机的变换
        current_drone_transform_.setOrigin(tf::Vector3(msg->pose.position.x,
                                                       msg->pose.position.y,
                                                       msg->pose.position.z));

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.orientation, q);
        // 反转四元数来修正旋转方向
        q = q.inverse();
        q.normalize();
        current_drone_transform_.setRotation(q);

        // 发布drone_frame的TF
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(current_drone_transform_, msg->header.stamp,
                                 "drone_init", "drone_frame"));

        has_drone_pose_ = true;
        last_drone_update_ = msg->header.stamp;

        ROS_DEBUG("Updated drone transform at time: %f", msg->header.stamp.toSec());
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!has_drone_pose_)
        {
            ROS_WARN_THROTTLE(1.0, "No drone pose available yet");
            return;
        }

        // 创建从drone_frame到lidar的变换
        tf::Transform lidar_transform;
        lidar_transform.setOrigin(tf::Vector3(0, 0, 0));

        // 设置绕X轴旋转180度的四元数
        tf::Quaternion lidar_rotation;
        lidar_rotation.setRPY(M_PI, 0, 0);
        lidar_transform.setRotation(lidar_rotation);

        // 使用雷达数据的时间戳发布TF
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(lidar_transform, msg->header.stamp,
                                 "drone_frame", "lidar"));

        // 重新发布drone_frame的TF，确保与雷达同步
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(current_drone_transform_, msg->header.stamp,
                                 "drone_init", "drone_frame"));

        ROS_DEBUG("Broadcasting TFs at lidar time: %f", msg->header.stamp.toSec());
    }

    void endGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!isValidPose(msg))
        {
            return;
        }

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.position.x,
                                        msg->pose.position.y,
                                        msg->pose.position.z));

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.orientation, q);
        q.normalize();
        transform.setRotation(q);

        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, msg->header.stamp,
                                 "drone_init", "goal_frame"));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    TFBroadcaster broadcaster;

    ros::spin();

    return 0;
}
