#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

class DroneTransform
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher init_pose_pub_;
    ros::Publisher end_goal_pub_;
    ros::Subscriber pose_gt_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber end_goal_sub_;

    bool has_initial_pose_;
    tf::Transform initial_transform_;

public:
    DroneTransform() : has_initial_pose_(false)
    {
        ROS_INFO("Initializing DroneTransform node...");

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/airsim_node/drone_1/drone_pose", 10);
        init_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/airsim_node/drone_1/drone_init", 10);
        end_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/airsim_node/drone_1/drone_end", 10);

        initial_pose_sub_ = nh_.subscribe("/airsim_node/initial_pose", 10, &DroneTransform::initialPoseCallback, this);
        end_goal_sub_ = nh_.subscribe("/airsim_node/end_goal", 10, &DroneTransform::endGoalCallback, this);

        ros::Duration(1.0).sleep();
        pose_gt_sub_ = nh_.subscribe("/airsim_node/drone_1/filtered_gps", 10, &DroneTransform::poseGtCallback, this);

        ROS_INFO("Subscribers and publishers initialized");
    }

    void initialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!has_initial_pose_)
        {
            has_initial_pose_ = true;
        }
        tf::Vector3 initial_position(msg->pose.position.x,
                                     msg->pose.position.y,
                                     msg->pose.position.z);
        tf::Quaternion initial_rotation(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        initial_rotation.normalize();

        initial_transform_.setOrigin(initial_position);
        initial_transform_.setRotation(initial_rotation);

        // 发布初始位姿（角度为0度）
        geometry_msgs::PoseStamped init_pose;
        init_pose.header.stamp = ros::Time::now();
        init_pose.header.frame_id = "drone_init";
        init_pose.pose.position.x = 0;
        init_pose.pose.position.y = 0;
        init_pose.pose.position.z = 0;

        tf::Quaternion zero_rotation(0, 0, 0, 1); // w=1 表示单位四元数
        tf::quaternionTFToMsg(zero_rotation, init_pose.pose.orientation);

        init_pose_pub_.publish(init_pose);
    }

    void endGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!has_initial_pose_)
        {
            ROS_WARN_THROTTLE(1.0, "Waiting for initial pose before transforming end goal...");
            return;
        }

        tf::Transform goal_transform;
        tf::Vector3 goal_position(msg->pose.position.x,
                                  msg->pose.position.y,
                                  msg->pose.position.z);
        tf::Quaternion goal_rotation(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        goal_rotation.normalize();

        goal_transform.setOrigin(goal_position);
        goal_transform.setRotation(goal_rotation);

        // 计算相对变换
        tf::Transform relative_transform = initial_transform_.inverse() * goal_transform;
        tf::Vector3 relative_position = relative_transform.getOrigin();

        // 创建输出消息
        geometry_msgs::PoseStamped end_goal;
        end_goal.header.stamp = ros::Time::now();
        end_goal.header.frame_id = "goal_frame";

        // 设置位置（注意z轴和y轴反向）
        end_goal.pose.position.x = relative_position.x();
        end_goal.pose.position.y = -relative_position.y();
        end_goal.pose.position.z = -relative_position.z();

        // 强制设置为0度
        tf::Quaternion zero_rotation(0, 0, 0, 1); // w=1 表示单位四元数
        tf::quaternionTFToMsg(zero_rotation, end_goal.pose.orientation);

        end_goal_pub_.publish(end_goal);
    }

    void poseGtCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!has_initial_pose_)
        {
            ROS_WARN_THROTTLE(1.0, "Waiting for initial pose...");
            return;
        }

        tf::Transform current_transform;
        tf::Vector3 current_position(msg->pose.position.x,
                                     msg->pose.position.y,
                                     msg->pose.position.z);
        tf::Quaternion current_rotation(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        current_rotation.normalize();

        current_transform.setOrigin(current_position);
        current_transform.setRotation(current_rotation);

        // 计算相对变换
        tf::Transform relative_transform = initial_transform_.inverse() * current_transform;
        tf::Vector3 relative_position = relative_transform.getOrigin();
        // tf::Quaternion relative_rotation = relative_transform.getRotation();
        // relative_rotation.normalize();
        // 创建输出消息
        geometry_msgs::PoseStamped drone_pose;
        drone_pose.header.stamp = ros::Time::now();
        drone_pose.header.frame_id = "drone_frame";

        // 设置位置（注意z轴和y轴反向）
        drone_pose.pose.position.x = relative_position.x();
        drone_pose.pose.position.y = -relative_position.y();
        drone_pose.pose.position.z = -relative_position.z();

        // 直接使用IMU的姿态数据
        drone_pose.pose.orientation = msg->pose.orientation;

        pose_pub_.publish(drone_pose);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_trans");
    DroneTransform transformer;
    ROS_INFO("Starting tf_trans node");
    ros::spin();
    return 0;
}
