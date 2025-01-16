#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <vector>

class PathPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher target_marker_pub_;
    ros::Subscriber pose_sub_;
    ros::ServiceServer start_service_;
    ros::Timer marker_timer_;

    bool is_started_ = false;

    // Target position
    const double target_x_ = 2.0;
    const double target_y_ = 1.0;
    const double target_z_ = 1.5;

    // Path parameters
    const int num_points_ = 50;
    const double min_height_ = 1.0;

    nav_msgs::Path current_path_;
    geometry_msgs::PoseStamped current_pose_;
    bool pose_received_ = false;

    bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        is_started_ = true;
        ROS_INFO("Received start command, drone will begin flying!");
        return true;
    }

    void generatePath()
    {
        if (!pose_received_ || !is_started_) // Only generate path if started
            return;

        current_path_.header.stamp = ros::Time::now();
        current_path_.header.frame_id = "drone_init";
        current_path_.poses.clear();

        double start_x = current_pose_.pose.position.x;
        double start_y = current_pose_.pose.position.y;
        double start_z = current_pose_.pose.position.z;

        // Generate smooth path points
        for (int i = 0; i <= num_points_; i++)
        {
            double t = static_cast<double>(i) / num_points_;

            // Use cubic interpolation for smooth acceleration and deceleration
            double s = t * t * (3 - 2 * t);

            geometry_msgs::PoseStamped pose;
            pose.header = current_path_.header;

            // Position interpolation
            pose.pose.position.x = start_x + (target_x_ - start_x) * s;
            pose.pose.position.y = start_y + (target_y_ - start_y) * s;

            // Add a height profile: first go up, then move to target
            double height_factor = sin(M_PI * s);
            double extra_height = 0.5 * height_factor; // Maximum 0.5m extra height
            pose.pose.position.z = std::max(min_height_,
                                            start_z + (target_z_ - start_z) * s + extra_height);

            // Calculate orientation to face the direction of motion
            double dx = (target_x_ - start_x);
            double dy = (target_y_ - start_y);
            double yaw = atan2(dy, dx);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            current_path_.poses.push_back(pose);
        }

        // Publish path
        path_pub_.publish(current_path_);

        // Publish target marker
        publishTargetMarker();
    }

    void publishTargetMarker()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "drone_init";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = target_x_;
        marker.pose.position.y = target_y_;
        marker.pose.position.z = target_z_;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0);

        target_marker_pub_.publish(marker);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
        generatePath();
    }

    void markerTimerCallback(const ros::TimerEvent &)
    {
        publishTargetMarker();
    }

    void run()
    {
        ros::spin();
    }

public:
    PathPlanner()
    {
        path_pub_ = nh_.advertise<nav_msgs::Path>("/airsim_node/drone_1/path", 1);
        target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/airsim_node/drone_1/target_marker", 1);
        pose_sub_ = nh_.subscribe("/airsim_node/drone_1/drone_pose", 1, &PathPlanner::poseCallback, this);
        start_service_ = nh_.advertiseService("/drone_1/start_flight", &PathPlanner::startCallback, this);
        marker_timer_ = nh_.createTimer(ros::Duration(0.1), &PathPlanner::markerTimerCallback, this);

        ROS_INFO("Path planner initialized with parameters:");
        ROS_INFO("Step size: %.2f", 1.00);
        ROS_INFO("Goal bias: %.2f", 0.20);
        ROS_INFO("Search radius: %.2f", 2.00);
        ROS_INFO("Max iterations: %d", 1000);

        publishTargetMarker();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    PathPlanner path_planner;
    path_planner.run();
    return 0;
}