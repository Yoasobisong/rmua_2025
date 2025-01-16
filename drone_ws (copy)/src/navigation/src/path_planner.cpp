#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

class PathPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_;
    ros::Publisher tree_pub_;

    // RRT* parameters
    double step_size_;     // 步长
    double goal_bias_;     // 目标偏置概率
    double search_radius_; // 搜索半径
    int max_iterations_;   // 最大迭代次数

    // Planning space bounds
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_;

    // Current state
    std::vector<visualization_msgs::Marker> obstacles_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped goal_pose_;
    bool has_goal_;
    bool has_current_pose_;

    // Random number generator
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_;

    struct Node
    {
        double x, y, z;
        Node *parent;
        double cost;
        std::vector<Node *> children;

        Node(double x_, double y_, double z_) : x(x_), y(y_), z(z_), parent(nullptr), cost(0.0) {}
    };

    std::vector<Node *> nodes_;

public:
    PathPlanner() : nh_("~"), gen_(rd_()), dis_(0.0, 1.0), has_goal_(false), has_current_pose_(false)
    {
        // Load parameters
        nh_.param("step_size", step_size_, 1.0);
        nh_.param("goal_bias", goal_bias_, 0.2);
        nh_.param("search_radius", search_radius_, 2.0);
        nh_.param("max_iterations", max_iterations_, 1000);

        // Planning space bounds
        nh_.param("x_min", x_min_, -50.0);
        nh_.param("x_max", x_max_, 50.0);
        nh_.param("y_min", y_min_, -50.0);
        nh_.param("y_max", y_max_, 50.0);
        nh_.param("z_min", z_min_, 0.0);
        nh_.param("z_max", z_max_, 10.0);

        // Setup ROS communication
        obstacle_sub_ = nh_.subscribe("/airsim_node/drone_1/obstacles/markers", 1,
                                      &PathPlanner::obstacleCallback, this);
        goal_sub_ = nh_.subscribe("/airsim_node/drone_1/move_base_simple/goal", 1,
                                  &PathPlanner::goalCallback, this);
        ros::NodeHandle nh; // 使用全局命名空间
        pose_sub_ = nh.subscribe("/airsim_node/drone_1/drone_pose", 1,
                                 &PathPlanner::poseCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/airsim_node/drone_1/path", 1);
        tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/airsim_node/drone_1/rrt_tree", 1);

        ROS_INFO("Path planner initialized with parameters:");
        ROS_INFO("Step size: %.2f", step_size_);
        ROS_INFO("Goal bias: %.2f", goal_bias_);
        ROS_INFO("Search radius: %.2f", search_radius_);
        ROS_INFO("Max iterations: %d", max_iterations_);
    }

    ~PathPlanner()
    {
        for (auto node : nodes_)
        {
            delete node;
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
        has_current_pose_ = true;
    }

    void obstacleCallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
    {
        obstacles_.clear();
        for (const auto &marker : msg->markers)
        {
            obstacles_.push_back(marker);
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!has_current_pose_)
        {
            ROS_WARN("No current pose available, cannot plan path");
            return;
        }

        // 保持目标点的高度在合理范围内
        goal_pose_ = *msg;

        // 如果目标点高度太低或太高，调整到合理范围
        if (goal_pose_.pose.position.z < z_min_)
        {
            goal_pose_.pose.position.z = z_min_ + 1.0; // 保持在最小高度以上1米
            ROS_WARN("Goal height adjusted to minimum safe height: %.2f", goal_pose_.pose.position.z);
        }
        else if (goal_pose_.pose.position.z > z_max_)
        {
            goal_pose_.pose.position.z = z_max_ - 1.0; // 保持在最大高度以下1米
            ROS_WARN("Goal height adjusted to maximum safe height: %.2f", goal_pose_.pose.position.z);
        }

        // 如果目标点高度接近当前高度（可能是2D设置的目标点），保持当前高度
        if (fabs(goal_pose_.pose.position.z) < 0.1)
        {
            goal_pose_.pose.position.z = current_pose_.pose.position.z;
            ROS_INFO("Maintaining current height for goal: %.2f", goal_pose_.pose.position.z);
        }

        has_goal_ = true;
        ROS_INFO("Received new goal, planning path...");
        planPath(); // 只在收到新目标点时规划一次路径
    }

    bool isCollisionFree(const Node *from, const Node *to)
    {
        // Simple collision check with obstacles
        for (const auto &obstacle : obstacles_)
        {
            // Get obstacle bounds
            double ox = obstacle.pose.position.x;
            double oy = obstacle.pose.position.y;
            double oz = obstacle.pose.position.z;
            double sx = obstacle.scale.x / 2;
            double sy = obstacle.scale.y / 2;
            double sz = obstacle.scale.z / 2;

            // Check if line intersects with obstacle box
            // This is a simplified collision check
            double dx = to->x - from->x;
            double dy = to->y - from->y;
            double dz = to->z - from->z;
            double length = sqrt(dx * dx + dy * dy + dz * dz);

            if (length < 0.001)
                continue;

            // Check multiple points along the line
            int check_points = 10;
            for (int i = 0; i <= check_points; i++)
            {
                double t = i / (double)check_points;
                double px = from->x + t * dx;
                double py = from->y + t * dy;
                double pz = from->z + t * dz;

                if (px >= ox - sx && px <= ox + sx &&
                    py >= oy - sy && py <= oy + sy &&
                    pz >= oz - sz && pz <= oz + sz)
                {
                    return false;
                }
            }
        }
        return true;
    }

    Node *getRandomNode()
    {
        if (dis_(gen_) < goal_bias_ && has_goal_)
        {
            return new Node(goal_pose_.pose.position.x,
                            goal_pose_.pose.position.y,
                            goal_pose_.pose.position.z);
        }

        return new Node(
            x_min_ + dis_(gen_) * (x_max_ - x_min_),
            y_min_ + dis_(gen_) * (y_max_ - y_min_),
            z_min_ + dis_(gen_) * (z_max_ - z_min_));
    }

    Node *getNearestNode(const Node *target)
    {
        Node *nearest = nullptr;
        double min_dist = std::numeric_limits<double>::max();

        for (const auto &node : nodes_)
        {
            double dist = std::sqrt(std::pow(node->x - target->x, 2) +
                                    std::pow(node->y - target->y, 2) +
                                    std::pow(node->z - target->z, 2));
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest = node;
            }
        }
        return nearest;
    }

    void planPath()
    {
        if (!has_goal_ || !has_current_pose_)
        {
            ROS_WARN("Cannot plan path: %s",
                     !has_goal_ ? "No goal set" : "No current pose available");
            return;
        }

        // Clear previous nodes
        for (auto node : nodes_)
        {
            delete node;
        }
        nodes_.clear();

        // Start from current drone position
        Node *start = new Node(current_pose_.pose.position.x,
                               current_pose_.pose.position.y,
                               current_pose_.pose.position.z);
        nodes_.push_back(start);

        // RRT* algorithm
        for (int i = 0; i < max_iterations_; i++)
        {
            // Get random node
            Node *random_node = getRandomNode();

            // Find nearest node in tree
            Node *nearest = getNearestNode(random_node);

            // Steer towards random node
            double dx = random_node->x - nearest->x;
            double dy = random_node->y - nearest->y;
            double dz = random_node->z - nearest->z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            Node *new_node;
            if (dist > step_size_)
            {
                new_node = new Node(
                    nearest->x + dx * step_size_ / dist,
                    nearest->y + dy * step_size_ / dist,
                    nearest->z + dz * step_size_ / dist);
            }
            else
            {
                new_node = random_node;
                random_node = nullptr;
            }

            // Check if path to new node is collision free
            if (isCollisionFree(nearest, new_node))
            {
                // Find best parent
                std::vector<Node *> nearby_nodes;
                for (auto node : nodes_)
                {
                    double dist = std::sqrt(std::pow(node->x - new_node->x, 2) +
                                            std::pow(node->y - new_node->y, 2) +
                                            std::pow(node->z - new_node->z, 2));
                    if (dist < search_radius_)
                    {
                        nearby_nodes.push_back(node);
                    }
                }

                Node *best_parent = nearest;
                double best_cost = nearest->cost + std::sqrt(std::pow(new_node->x - nearest->x, 2) +
                                                             std::pow(new_node->y - nearest->y, 2) +
                                                             std::pow(new_node->z - nearest->z, 2));

                for (auto node : nearby_nodes)
                {
                    if (isCollisionFree(node, new_node))
                    {
                        double cost = node->cost + std::sqrt(std::pow(new_node->x - node->x, 2) +
                                                             std::pow(new_node->y - node->y, 2) +
                                                             std::pow(new_node->z - node->z, 2));
                        if (cost < best_cost)
                        {
                            best_cost = cost;
                            best_parent = node;
                        }
                    }
                }

                // Add new node to tree
                new_node->parent = best_parent;
                new_node->cost = best_cost;
                best_parent->children.push_back(new_node);
                nodes_.push_back(new_node);

                // Rewire nearby nodes
                for (auto node : nearby_nodes)
                {
                    if (node != best_parent)
                    {
                        double cost = new_node->cost + std::sqrt(std::pow(node->x - new_node->x, 2) +
                                                                 std::pow(node->y - new_node->y, 2) +
                                                                 std::pow(node->z - new_node->z, 2));
                        if (cost < node->cost && isCollisionFree(new_node, node))
                        {
                            // Remove node from old parent's children
                            auto &children = node->parent->children;
                            children.erase(std::remove(children.begin(), children.end(), node),
                                           children.end());

                            // Update node's parent
                            node->parent = new_node;
                            node->cost = cost;
                            new_node->children.push_back(node);

                            // Update costs of all descendants
                            std::vector<Node *> to_update = {node};
                            while (!to_update.empty())
                            {
                                Node *current = to_update.back();
                                to_update.pop_back();
                                for (auto child : current->children)
                                {
                                    child->cost = current->cost +
                                                  std::sqrt(std::pow(child->x - current->x, 2) +
                                                            std::pow(child->y - current->y, 2) +
                                                            std::pow(child->z - current->z, 2));
                                    to_update.push_back(child);
                                }
                            }
                        }
                    }
                }
            }

            if (random_node)
                delete random_node;

            // Check if we're close enough to goal
            double dist_to_goal = std::sqrt(std::pow(new_node->x - goal_pose_.pose.position.x, 2) +
                                            std::pow(new_node->y - goal_pose_.pose.position.y, 2) +
                                            std::pow(new_node->z - goal_pose_.pose.position.z, 2));
            if (dist_to_goal < step_size_)
            {
                break;
            }
        }

        publishPath();
        publishTree();
    }

    void publishPath()
    {
        // Find node closest to goal
        Node *closest_to_goal = nullptr;
        double min_dist = std::numeric_limits<double>::max();
        for (const auto &node : nodes_)
        {
            double dist = std::sqrt(std::pow(node->x - goal_pose_.pose.position.x, 2) +
                                    std::pow(node->y - goal_pose_.pose.position.y, 2) +
                                    std::pow(node->z - goal_pose_.pose.position.z, 2));
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_to_goal = node;
            }
        }

        // Create path message
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "drone_init";
        path_msg.header.stamp = ros::Time::now();

        // Backtrack from goal to start
        std::vector<Node *> path_nodes;
        Node *current = closest_to_goal;
        while (current != nullptr)
        {
            path_nodes.push_back(current);
            current = current->parent;
        }

        // Add poses to path message in correct order
        for (auto it = path_nodes.rbegin(); it != path_nodes.rend(); ++it)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = (*it)->x;
            pose.pose.position.y = (*it)->y;
            pose.pose.position.z = (*it)->z;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_.publish(path_msg);
    }

    void publishTree()
    {
        visualization_msgs::MarkerArray marker_array;

        // Create marker for nodes
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = "drone_init";
        node_marker.header.stamp = ros::Time::now();
        node_marker.ns = "rrt_nodes";
        node_marker.id = 0;
        node_marker.type = visualization_msgs::Marker::POINTS;
        node_marker.action = visualization_msgs::Marker::ADD;
        node_marker.scale.x = 0.1;
        node_marker.scale.y = 0.1;
        node_marker.color.r = 0.0;
        node_marker.color.g = 1.0;
        node_marker.color.b = 0.0;
        node_marker.color.a = 1.0;

        // Create marker for edges
        visualization_msgs::Marker edge_marker;
        edge_marker.header.frame_id = "drone_init";
        edge_marker.header.stamp = ros::Time::now();
        edge_marker.ns = "rrt_edges";
        edge_marker.id = 1;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.scale.x = 0.02;
        edge_marker.color.r = 0.0;
        edge_marker.color.g = 0.8;
        edge_marker.color.b = 0.0;
        edge_marker.color.a = 0.5;

        // Add all nodes and edges
        for (const auto &node : nodes_)
        {
            geometry_msgs::Point p;
            p.x = node->x;
            p.y = node->y;
            p.z = node->z;
            node_marker.points.push_back(p);

            if (node->parent)
            {
                edge_marker.points.push_back(p);
                p.x = node->parent->x;
                p.y = node->parent->y;
                p.z = node->parent->z;
                edge_marker.points.push_back(p);
            }
        }

        marker_array.markers.push_back(node_marker);
        marker_array.markers.push_back(edge_marker);
        tree_pub_.publish(marker_array);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    PathPlanner planner;
    ros::spin();
    return 0;
}
