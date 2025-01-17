#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

class TrajectoryPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher global_path_pub_;

    // 参数
    double update_rate_; // 规划更新频率

    // 目标点
    geometry_msgs::PoseStamped goal_pose_;
    bool has_goal_;

    // 全局路径
    nav_msgs::Path global_path_;

    // 当前位置
    geometry_msgs::PoseStamped current_pose_;

    /**
     * 生成全局路径（直线）
     */
    void generateGlobalPath()
    {
        if (!has_goal_)
            return;

        global_path_.header.stamp = ros::Time::now();
        global_path_.header.frame_id = "drone_init";
        global_path_.poses.clear();

        // 添加起点
        global_path_.poses.push_back(current_pose_);

        // 添加终点
        global_path_.poses.push_back(goal_pose_);

        // 发布全局路径
        global_path_pub_.publish(global_path_);
    }

    /**
     * 目标点回调函数
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        goal_pose_ = *goal_msg;
        has_goal_ = true;
        generateGlobalPath();
    }

    /**
     * 位姿回调函数
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        current_pose_ = *pose_msg;
        if (has_goal_)
        {
            generateGlobalPath();
        }
    }

public:
    TrajectoryPlanner() : nh_("~"), has_goal_(false)
    {
        // 加载参数
        nh_.param("update_rate", update_rate_, 10.0);

        // 订阅目标点
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                                  &TrajectoryPlanner::goalCallback, this);

        // 订阅无人机位姿
        pose_sub_ = nh_.subscribe("/airsim_node/drone_1/pose", 1,
                                  &TrajectoryPlanner::poseCallback, this);

        // 发布全局路径
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_trajectory", 1);
    }

    void run()
    {
        ros::Rate rate(update_rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_planner");
    TrajectoryPlanner planner;
    planner.run();
    return 0;
}