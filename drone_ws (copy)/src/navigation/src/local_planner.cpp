#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <tf/tf.h>
#include <angles/angles.h>

class LocalPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_path_pub_;
    ros::Publisher trajectory_pub_;

    nav_msgs::Path global_path_;
    geometry_msgs::PoseStamped current_pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_;

    // DWA参数
    double max_vel_;        // 最大速度
    double max_accel_;      // 最大加速度
    double max_yaw_rate_;   // 最大偏航角速度
    double max_yaw_accel_;  // 最大偏航角加速度
    double vel_resolution_; // 速度分辨率
    double yaw_resolution_; // 偏航角分辨率
    double dt_;             // 时间步长
    double predict_time_;   // 轨迹预测时间
    double goal_tolerance_; // 目标容差

    // 局部规划范围
    double forward_range_;  // 前向距离
    double side_range_;     // 侧向距离
    double backward_range_; // 后向距离

    // 避障参数
    double min_obstacle_dist_; // 最小避障距离
    double safety_margin_;     // 安全缓冲区

    // 代价函数权重
    double obstacle_weight_; // 障碍物代价权重
    double velocity_weight_; // 速度代价权重
    double goal_weight_;     // 目标点代价权重
    double heading_weight_;  // 航向代价权重

    bool has_global_path_;
    bool has_current_pose_;
    int current_waypoint_;

public:
    LocalPlanner() : nh_("~"), obstacles_(new pcl::PointCloud<pcl::PointXYZ>),
                     has_global_path_(false), has_current_pose_(false), current_waypoint_(0)
    {
        // 加载DWA参数
        nh_.param("velocity/max_vel", max_vel_, 3.0);
        nh_.param("velocity/max_accel", max_accel_, 2.0);
        nh_.param("velocity/vel_resolution", vel_resolution_, 0.1);

        nh_.param("yaw/max_rate", max_yaw_rate_, 1.5);
        nh_.param("yaw/max_accel", max_yaw_accel_, 1.0);
        nh_.param("yaw/resolution", yaw_resolution_, 0.1);

        nh_.param("trajectory/dt", dt_, 0.1);
        nh_.param("trajectory/predict_time", predict_time_, 5.0);
        nh_.param("trajectory/goal_tolerance", goal_tolerance_, 0.5);

        // 加载局部规划范围
        nh_.param("local_range/forward", forward_range_, 8.0);
        nh_.param("local_range/side", side_range_, 6.0);
        nh_.param("local_range/backward", backward_range_, 4.0);

        // 加载避障参数
        nh_.param("obstacle/min_distance", min_obstacle_dist_, 2.0);
        nh_.param("obstacle/safety_margin", safety_margin_, 1.0);

        // 加载代价函数权重
        nh_.param("cost_weights/obstacle", obstacle_weight_, 2.0);
        nh_.param("cost_weights/velocity", velocity_weight_, 1.0);
        nh_.param("cost_weights/goal", goal_weight_, 1.0);
        nh_.param("cost_weights/heading", heading_weight_, 1.0);

        // 设置订阅器和发布器
        global_path_sub_ = nh_.subscribe("/airsim_node/drone_1/path", 1,
                                         &LocalPlanner::globalPathCallback, this);
        obstacles_sub_ = nh_.subscribe("/airsim_node/drone_1/pointcloud/obstacles", 1,
                                       &LocalPlanner::obstaclesCallback, this);
        pose_sub_ = nh_.subscribe("/airsim_node/drone_1/drone_pose", 1,
                                  &LocalPlanner::poseCallback, this);
        local_path_pub_ = nh_.advertise<nav_msgs::Path>("/airsim_node/drone_1/local_path", 1);
        trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/airsim_node/drone_1/dwa_trajectories", 1);

        ROS_INFO("Local planner initialized with parameters:");
        ROS_INFO("Max velocity: %.2f m/s", max_vel_);
        ROS_INFO("Forward range: %.2f m", forward_range_);
        ROS_INFO("Min obstacle distance: %.2f m", min_obstacle_dist_);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        global_path_ = *msg;
        has_global_path_ = true;
        current_waypoint_ = 0;
        ROS_INFO("Received new global path with %zu waypoints", msg->poses.size());
    }

    void obstaclesCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::fromROSMsg(*msg, *obstacles_);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
        has_current_pose_ = true;
    }

    // 计算两点间距离
    double getDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) +
                         std::pow(p1.y - p2.y, 2) +
                         std::pow(p1.z - p2.z, 2));
    }

    // 更新当前目标点
    void updateCurrentWaypoint()
    {
        if (!has_global_path_ || current_waypoint_ >= global_path_.poses.size())
        {
            return;
        }

        while (current_waypoint_ < global_path_.poses.size())
        {
            double dist = getDistance(current_pose_.pose.position,
                                      global_path_.poses[current_waypoint_].pose.position);
            if (dist > goal_tolerance_)
            {
                break;
            }
            current_waypoint_++;
        }
    }

    // 生成DWA轨迹
    std::vector<nav_msgs::Path> generateTrajectories()
    {
        std::vector<nav_msgs::Path> trajectories;

        // 当前速度范围
        double min_vel = std::max(-max_vel_, -max_accel_ * dt_);
        double max_vel = std::min(max_vel_, max_accel_ * dt_);

        // 当前偏航角速度范围
        double min_yaw_rate = std::max(-max_yaw_rate_, -max_yaw_accel_ * dt_);
        double max_yaw_rate = std::min(max_yaw_rate_, max_yaw_accel_ * dt_);

        // 遍历所有可能的速度组合
        for (double v = min_vel; v <= max_vel; v += vel_resolution_)
        {
            for (double w = min_yaw_rate; w <= max_yaw_rate; w += yaw_resolution_)
            {
                nav_msgs::Path trajectory;
                trajectory.header.frame_id = "drone_init";
                trajectory.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped pose = current_pose_;
                double yaw = tf::getYaw(pose.pose.orientation);

                // 预测轨迹
                for (double t = 0; t <= predict_time_; t += dt_)
                {
                    yaw += w * dt_;
                    pose.pose.position.x += v * cos(yaw) * dt_;
                    pose.pose.position.y += v * sin(yaw) * dt_;
                    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                    trajectory.poses.push_back(pose);
                }
                trajectories.push_back(trajectory);
            }
        }
        return trajectories;
    }

    // 评估轨迹
    double evaluateTrajectory(const nav_msgs::Path &trajectory)
    {
        if (current_waypoint_ >= global_path_.poses.size())
        {
            return -1000.0;
        }

        double score = 0.0;

        // 目标导向得分
        geometry_msgs::Point goal = global_path_.poses[current_waypoint_].pose.position;
        double goal_dist = getDistance(trajectory.poses.back().pose.position, goal);
        score -= goal_weight_ * goal_dist;

        // 速度得分（鼓励更快的速度）
        double vel = getDistance(trajectory.poses[0].pose.position,
                                 trajectory.poses[1].pose.position) /
                     dt_;
        score += velocity_weight_ * (vel / max_vel_);

        // 航向得分（鼓励朝向目标点）
        double target_yaw = atan2(goal.y - current_pose_.pose.position.y,
                                  goal.x - current_pose_.pose.position.x);
        double current_yaw = tf::getYaw(trajectory.poses.back().pose.orientation);
        double yaw_diff = std::abs(angles::shortest_angular_distance(current_yaw, target_yaw));
        score -= heading_weight_ * yaw_diff;

        // 障碍物避免得分
        for (const auto &pose : trajectory.poses)
        {
            for (const auto &point : obstacles_->points)
            {
                double dist = std::sqrt(std::pow(pose.pose.position.x - point.x, 2) +
                                        std::pow(pose.pose.position.y - point.y, 2) +
                                        std::pow(pose.pose.position.z - point.z, 2));

                if (dist < min_obstacle_dist_)
                {
                    score -= obstacle_weight_ * (min_obstacle_dist_ - dist);
                }
                else if (dist < min_obstacle_dist_ + safety_margin_)
                {
                    score -= obstacle_weight_ * (min_obstacle_dist_ + safety_margin_ - dist) * 0.5;
                }
            }
        }

        return score;
    }

    // 选择最佳轨迹
    nav_msgs::Path selectBestTrajectory()
    {
        std::vector<nav_msgs::Path> trajectories = generateTrajectories();
        nav_msgs::Path best_trajectory;
        double best_score = -std::numeric_limits<double>::infinity();

        for (const auto &trajectory : trajectories)
        {
            double score = evaluateTrajectory(trajectory);
            if (score > best_score)
            {
                best_score = score;
                best_trajectory = trajectory;
            }
        }

        // 可视化所有轨迹
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        for (const auto &trajectory : trajectories)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "drone_init";
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectories";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.1;
            marker.color.a = 0.3;
            marker.color.r = (trajectory == best_trajectory) ? 0.0 : 0.8;
            marker.color.g = (trajectory == best_trajectory) ? 1.0 : 0.2;
            marker.color.b = 0.0;

            for (const auto &pose : trajectory.poses)
            {
                geometry_msgs::Point p = pose.pose.position;
                marker.points.push_back(p);
            }
            marker_array.markers.push_back(marker);
        }
        trajectory_pub_.publish(marker_array);

        return best_trajectory;
    }

    void run()
    {
        double update_rate;
        nh_.param("update_rate", update_rate, 20.0);
        ros::Rate rate(update_rate);

        ROS_INFO("Local planner running at %.1f Hz", update_rate);

        while (ros::ok())
        {
            if (has_global_path_ && has_current_pose_)
            {
                updateCurrentWaypoint();
                if (current_waypoint_ < global_path_.poses.size())
                {
                    nav_msgs::Path local_path = selectBestTrajectory();
                    if (!local_path.poses.empty())
                    {
                        local_path.header.stamp = ros::Time::now();
                        local_path.header.frame_id = "drone_init";
                        local_path_pub_.publish(local_path);
                    }
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");
    LocalPlanner local_planner;
    local_planner.run();
    return 0;
}