#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

class EgoPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_map_pub_;
    ros::Publisher optimized_traj_pub_;
    ros::Publisher debug_pub_;

    // TF相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 参数
    double planning_horizon_;   // 局部规划范围
    double update_rate_;        // 规划更新频率
    double min_altitude_;       // 最小飞行高度
    double max_altitude_;       // 最大飞行高度
    double safe_distance_;      // 安全距离
    double resolution_;         // 地图分辨率
    int max_iter_;              // 最大迭代次数
    double weight_smooth_;      // 平滑项权重
    double weight_collision_;   // 碰撞项权重
    double weight_feasibility_; // 可行性权重

    // 状态变量
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped goal_pose_;
    bool has_goal_;
    bool has_odom_;

    // 局部地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;

    // 轨迹优化相关
    struct TrajectoryPoint
    {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector3d acc;
    };
    std::vector<TrajectoryPoint> traj_points_;

public:
    EgoPlanner() : nh_("~"), tf_listener_(tf_buffer_)
    {
        // 获取参数
        nh_.param("planning_horizon", planning_horizon_, 10.0);
        nh_.param("update_rate", update_rate_, 10.0);
        nh_.param("min_altitude", min_altitude_, 1.0);
        nh_.param("max_altitude", max_altitude_, 10.0);
        nh_.param("safe_distance", safe_distance_, 1.0);
        nh_.param("resolution", resolution_, 0.2);
        nh_.param("max_iter", max_iter_, 100);
        nh_.param("weight_smooth", weight_smooth_, 1.0);
        nh_.param("weight_collision", weight_collision_, 10.0);
        nh_.param("weight_feasibility", weight_feasibility_, 5.0);

        // 初始化点云
        local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);

        // 初始化订阅者和发布者
        cloud_sub_ = nh_.subscribe("cloud", 1, &EgoPlanner::cloudCallback, this);
        goal_sub_ = nh_.subscribe("goal", 1, &EgoPlanner::goalCallback, this);
        pose_sub_ = nh_.subscribe("pose", 1, &EgoPlanner::poseCallback, this);

        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("local_map", 1);
        optimized_traj_pub_ = nh_.advertise<nav_msgs::Path>("optimized_trajectory", 1);
        debug_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("debug_markers", 1);

        has_goal_ = false;
        has_odom_ = false;

        ROS_INFO("Ego planner initialized with parameters:");
        ROS_INFO("  planning_horizon: %.2f", planning_horizon_);
        ROS_INFO("  safe_distance: %.2f", safe_distance_);
        ROS_INFO("  resolution: %.2f", resolution_);
        ROS_INFO("  max_iter: %d", max_iter_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
    {
        ROS_INFO_THROTTLE(1.0, "[DEBUG] Received point cloud with frame_id: %s, width: %d, height: %d",
                          cloud_msg->header.frame_id.c_str(), cloud_msg->width, cloud_msg->height);

        // 转换点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 清空局部地图
        local_map_->clear();

        // 获取当前位置（在drone_frame下为原点）
        Eigen::Vector3d current_pos(0, 0, 0);

        // 提取局部范围内的点云
        for (const auto &pt : cloud->points)
        {
            Eigen::Vector3d point(pt.x, pt.y, pt.z);
            if ((point - current_pos).norm() < planning_horizon_)
            {
                local_map_->push_back(pt);
            }
        }

        ROS_INFO_THROTTLE(1.0, "[DEBUG] Local map contains %d points, planning_horizon: %.2f",
                          local_map_->size(), planning_horizon_);

        // 更新KD树
        kdtree_->setInputCloud(local_map_);

        // 发布局部地图用于可视化
        sensor_msgs::PointCloud2 local_map_msg;
        pcl::toROSMsg(*local_map_, local_map_msg);
        local_map_msg.header = cloud_msg->header;
        local_map_msg.header.frame_id = "drone_frame";
        local_map_pub_.publish(local_map_msg);
        ROS_INFO_THROTTLE(1.0, "[DEBUG] Published local map with frame_id: %s",
                          local_map_msg.header.frame_id.c_str());

        // 如果有目标点，进行路径规划
        if (has_goal_ && has_odom_)
        {
            ROS_INFO("[DEBUG] Starting path planning with goal and odom");
            planTrajectory();
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "[DEBUG] Waiting for goal and odom: has_goal_=%d, has_odom_=%d",
                              has_goal_, has_odom_);
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ROS_INFO("[DEBUG] Received goal with frame_id: %s, position: (%.2f, %.2f, %.2f)",
                 msg->header.frame_id.c_str(),
                 msg->pose.position.x,
                 msg->pose.position.y,
                 msg->pose.position.z);

        try
        {
            // 将目标点转换到drone_frame坐标系
            geometry_msgs::PoseStamped goal_frame;

            // 等待转换可用
            if (!tf_buffer_.canTransform("drone_frame", msg->header.frame_id, ros::Time(0)))
            {
                ROS_WARN_THROTTLE(1.0, "[DEBUG] Transform from %s to drone_frame not available yet",
                                  msg->header.frame_id.c_str());
                return;
            }

            // 执行转换
            goal_frame = tf_buffer_.transform(*msg, "drone_frame", ros::Duration(0.1));
            goal_pose_ = goal_frame;
            has_goal_ = true;

            ROS_INFO("[DEBUG] Transformed goal in drone_frame: (%.2f, %.2f, %.2f)",
                     goal_pose_.pose.position.x,
                     goal_pose_.pose.position.y,
                     goal_pose_.pose.position.z);

            if (has_odom_)
            {
                ROS_INFO("[DEBUG] Has odom, starting path planning");
                planTrajectory();
            }
            else
            {
                ROS_INFO("[DEBUG] No odom yet, waiting for odom data");
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("[DEBUG] Failed to transform goal pose: %s", ex.what());
            return;
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ROS_INFO_THROTTLE(1.0, "[DEBUG] Received pose with frame_id: %s, position: (%.2f, %.2f, %.2f)",
                          msg->header.frame_id.c_str(),
                          msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z);

        // 在drone_frame下，当前位置就是原点
        current_pose_.header.frame_id = "drone_frame";
        current_pose_.header.stamp = msg->header.stamp;
        current_pose_.pose.position.x = 0;
        current_pose_.pose.position.y = 0;
        current_pose_.pose.position.z = 0;
        current_pose_.pose.orientation.w = 1.0;
        has_odom_ = true;
        ROS_INFO_THROTTLE(1.0, "[DEBUG] Updated current pose in drone_frame");
    }

    void planTrajectory()
    {
        if (!has_goal_ || !has_odom_)
        {
            ROS_WARN("[DEBUG] Cannot plan trajectory: has_goal_=%d, has_odom_=%d",
                     has_goal_, has_odom_);
            return;
        }

        ROS_INFO("[DEBUG] Starting trajectory planning...");

        // 初始化轨迹
        initializeTrajectory();

        // 优化轨迹
        optimizeTrajectory();

        // 发布优化后的轨迹
        publishOptimizedTrajectory();
    }

    void initializeTrajectory()
    {
        traj_points_.clear();

        // 获取起点（在drone_frame下为原点）
        Eigen::Vector3d start_pos(0, 0, 0);

        // 获取终点
        Eigen::Vector3d end_pos(
            goal_pose_.pose.position.x,
            goal_pose_.pose.position.y,
            goal_pose_.pose.position.z);

        // 计算轨迹点数量
        int num_points = std::ceil((end_pos - start_pos).norm() / resolution_);
        num_points = std::max(num_points, 5); // 至少5个点

        ROS_INFO("Initializing trajectory with %d points", num_points);

        // 初始化为直线轨迹
        for (int i = 0; i <= num_points; i++)
        {
            double t = static_cast<double>(i) / num_points;
            TrajectoryPoint point;
            point.pos = start_pos * (1 - t) + end_pos * t;
            point.vel = (end_pos - start_pos) / num_points; // 简单速度估计
            point.acc.setZero();
            traj_points_.push_back(point);
        }
    }

    double calculateTrajectoryCost()
    {
        double cost = 0.0;

        // 1. 平滑性代价
        for (size_t i = 1; i < traj_points_.size() - 1; i++)
        {
            Eigen::Vector3d acc = traj_points_[i + 1].pos - 2 * traj_points_[i].pos + traj_points_[i - 1].pos;
            cost += weight_smooth_ * acc.squaredNorm();
        }

        // 2. 碰撞代价
        for (const auto &point : traj_points_)
        {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            pcl::PointXYZ searchPoint;
            searchPoint.x = point.pos.x();
            searchPoint.y = point.pos.y();
            searchPoint.z = point.pos.z();

            if (kdtree_->radiusSearch(searchPoint, safe_distance_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                // 如果在安全距离内有障碍物，增加代价
                for (const auto &dist : pointRadiusSquaredDistance)
                {
                    cost += weight_collision_ * (safe_distance_ - std::sqrt(dist));
                }
            }
        }

        // 3. 可行性代价（高度限制）
        for (const auto &point : traj_points_)
        {
            if (point.pos.z() < min_altitude_)
            {
                cost += weight_feasibility_ * std::pow(min_altitude_ - point.pos.z(), 2);
            }
            else if (point.pos.z() > max_altitude_)
            {
                cost += weight_feasibility_ * std::pow(point.pos.z() - max_altitude_, 2);
            }
        }

        return cost;
    }

    void optimizeTrajectory()
    {
        double step_size = 0.1;
        double prev_cost = calculateTrajectoryCost();

        for (int iter = 0; iter < max_iter_; iter++)
        {
            // 保存当前轨迹点
            std::vector<TrajectoryPoint> prev_points = traj_points_;

            // 对每个中间点进行优化
            for (size_t i = 1; i < traj_points_.size() - 1; i++)
            {
                // 在x, y, z方向上分别优化
                for (int dim = 0; dim < 3; dim++)
                {
                    // 尝试向正方向移动
                    traj_points_[i].pos[dim] += step_size;
                    double cost_plus = calculateTrajectoryCost();

                    // 尝试向负方向移动
                    traj_points_[i].pos[dim] -= 2 * step_size;
                    double cost_minus = calculateTrajectoryCost();

                    // 选择代价最小的方向
                    if (cost_plus < prev_cost && cost_plus <= cost_minus)
                    {
                        traj_points_[i].pos[dim] += step_size;
                        prev_cost = cost_plus;
                    }
                    else if (cost_minus < prev_cost)
                    {
                        prev_cost = cost_minus;
                    }
                    else
                    {
                        traj_points_[i].pos[dim] += step_size;
                    }
                }
            }

            // 更新速度和加速度
            for (size_t i = 0; i < traj_points_.size() - 1; i++)
            {
                traj_points_[i].vel = (traj_points_[i + 1].pos - traj_points_[i].pos) / resolution_;
            }
            traj_points_.back().vel.setZero();

            // 检查是否收敛
            double change = 0.0;
            for (size_t i = 0; i < traj_points_.size(); i++)
            {
                change += (traj_points_[i].pos - prev_points[i].pos).norm();
            }

            if (change < 0.001)
            {
                ROS_INFO("Trajectory optimization converged after %d iterations", iter + 1);
                break;
            }
        }
    }

    void publishOptimizedTrajectory()
    {
        nav_msgs::Path path;
        path.header.frame_id = "drone_frame";
        path.header.stamp = ros::Time::now();

        for (const auto &point : traj_points_)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = point.pos.x();
            pose.pose.position.y = point.pos.y();
            pose.pose.position.z = point.pos.z();

            // 计算朝向（使用速度方向）
            if (point.vel.norm() > 0.1)
            {
                Eigen::Vector3d dir = point.vel.normalized();
                double yaw = std::atan2(dir.y(), dir.x());
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();
            }
            else
            {
                pose.pose.orientation.w = 1.0;
            }

            path.poses.push_back(pose);
        }

        optimized_traj_pub_.publish(path);
        ROS_INFO("Published optimized trajectory with %zu points", path.poses.size());

        // 发布调试标记
        publishDebugMarkers();
    }

    void publishDebugMarkers()
    {
        visualization_msgs::MarkerArray markers;

        // 轨迹点标记
        visualization_msgs::Marker points;
        points.header.frame_id = "drone_frame";
        points.header.stamp = ros::Time::now();
        points.ns = "trajectory_points";
        points.action = visualization_msgs::Marker::ADD;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        points.id = 0;
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        points.scale.z = 0.2;
        points.color.r = 1.0;
        points.color.g = 0.0;
        points.color.b = 0.0;
        points.color.a = 1.0;

        for (const auto &point : traj_points_)
        {
            geometry_msgs::Point p;
            p.x = point.pos.x();
            p.y = point.pos.y();
            p.z = point.pos.z();
            points.points.push_back(p);
        }

        markers.markers.push_back(points);

        // 速度箭头标记
        visualization_msgs::Marker velocities;
        velocities.header = points.header;
        velocities.ns = "velocities";
        velocities.action = visualization_msgs::Marker::ADD;
        velocities.type = visualization_msgs::Marker::ARROW;
        velocities.id = 1;
        velocities.scale.x = 0.1; // 箭头轴直径
        velocities.scale.y = 0.2; // 箭头头部直径
        velocities.scale.z = 0.0;
        velocities.color.r = 0.0;
        velocities.color.g = 1.0;
        velocities.color.b = 0.0;
        velocities.color.a = 1.0;

        for (size_t i = 0; i < traj_points_.size(); i += 2)
        {
            if (traj_points_[i].vel.norm() > 0.1)
            {
                visualization_msgs::Marker arrow = velocities;
                arrow.id = i + 100;

                geometry_msgs::Point start, end;
                start.x = traj_points_[i].pos.x();
                start.y = traj_points_[i].pos.y();
                start.z = traj_points_[i].pos.z();

                Eigen::Vector3d vel_dir = traj_points_[i].vel.normalized();
                end.x = start.x + vel_dir.x();
                end.y = start.y + vel_dir.y();
                end.z = start.z + vel_dir.z();

                arrow.points.push_back(start);
                arrow.points.push_back(end);

                markers.markers.push_back(arrow);
            }
        }

        debug_pub_.publish(markers);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_planner");
    EgoPlanner planner;
    ros::spin();
    return 0;
}