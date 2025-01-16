#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <airsim_ros/RotorPWM.h>
#include <tf/tf.h>
#include <cmath>
#include <Eigen/Dense>

class MotionController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber local_path_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher pwm_pub_;

    // 控制参数
    double max_pwm_;        // 最大PWM值
    double min_pwm_;        // 最小PWM值
    double hover_pwm_;      // 悬停PWM值
    double pos_tolerance_;  // 位置容差
    double yaw_tolerance_;  // 偏航角容差
    double lookahead_dist_; // 前瞻距离

    // PID参数
    struct PIDGains
    {
        double kp, ki, kd;
    };
    PIDGains xy_gains_;  // 水平位置PID
    PIDGains z_gains_;   // 高度PID
    PIDGains yaw_gains_; // 偏航角PID

    // PID积分项
    Eigen::Vector3d pos_integral_;
    double yaw_integral_;

    // 当前状态
    geometry_msgs::PoseStamped current_pose_;
    sensor_msgs::Imu current_imu_;
    nav_msgs::Path current_path_;
    bool has_path_;
    bool has_pose_;
    bool has_imu_;
    int current_target_idx_;

    // 上一次误差（用于PID的微分项）
    Eigen::Vector3d last_pos_error_;
    double last_yaw_error_;
    ros::Time last_time_;

public:
    MotionController() : nh_("~"), has_path_(false), has_pose_(false), has_imu_(false),
                         current_target_idx_(0), pos_integral_(Eigen::Vector3d::Zero()),
                         yaw_integral_(0), last_pos_error_(Eigen::Vector3d::Zero()),
                         last_yaw_error_(0)
    {
        // 加载参数
        nh_.param("max_pwm", max_pwm_, 1.0);
        nh_.param("min_pwm", min_pwm_, 0.0);
        nh_.param("hover_pwm", hover_pwm_, 0.5);
        nh_.param("pos_tolerance", pos_tolerance_, 0.2);
        nh_.param("yaw_tolerance", yaw_tolerance_, 0.1);
        nh_.param("lookahead_dist", lookahead_dist_, 1.0);

        // 加载PID参数
        nh_.param("xy_kp", xy_gains_.kp, 0.5);
        nh_.param("xy_ki", xy_gains_.ki, 0.01);
        nh_.param("xy_kd", xy_gains_.kd, 0.1);

        nh_.param("z_kp", z_gains_.kp, 0.5);
        nh_.param("z_ki", z_gains_.ki, 0.01);
        nh_.param("z_kd", z_gains_.kd, 0.1);

        nh_.param("yaw_kp", yaw_gains_.kp, 0.5);
        nh_.param("yaw_ki", yaw_gains_.ki, 0.01);
        nh_.param("yaw_kd", yaw_gains_.kd, 0.1);

        // 设置订阅器和发布器
        local_path_sub_ = nh_.subscribe("/airsim_node/drone_1/local_path", 1,
                                        &MotionController::pathCallback, this);
        pose_sub_ = nh_.subscribe("/airsim_node/drone_1/drone_pose", 1,
                                  &MotionController::poseCallback, this);
        imu_sub_ = nh_.subscribe("/airsim_node/drone_1/imu/imu", 1,
                                 &MotionController::imuCallback, this);
        pwm_pub_ = nh_.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);

        last_time_ = ros::Time::now();
        ROS_INFO("Motion controller initialized with PWM control");
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        current_path_ = *msg;
        has_path_ = true;
        current_target_idx_ = 0;
        ROS_INFO("Received new path with %zu waypoints", msg->poses.size());
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
        has_pose_ = true;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        current_imu_ = *msg;
        has_imu_ = true;
    }

    double getDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) +
                         std::pow(p1.y - p2.y, 2) +
                         std::pow(p1.z - p2.z, 2));
    }

    int findLookaheadPoint()
    {
        if (current_target_idx_ >= current_path_.poses.size())
        {
            return current_path_.poses.size() - 1;
        }

        double accumulated_dist = 0.0;
        int i = current_target_idx_;

        while (i < current_path_.poses.size() - 1)
        {
            accumulated_dist += getDistance(current_path_.poses[i].pose.position,
                                            current_path_.poses[i + 1].pose.position);
            if (accumulated_dist >= lookahead_dist_)
            {
                break;
            }
            i++;
        }

        return i;
    }

    // PID控制器
    Eigen::Vector3d computePIDControl(const Eigen::Vector3d &error,
                                      Eigen::Vector3d &integral,
                                      const Eigen::Vector3d &last_error,
                                      const PIDGains &gains,
                                      double dt)
    {
        integral += error * dt;
        Eigen::Vector3d derivative = (error - last_error) / dt;
        return gains.kp * error + gains.ki * integral + gains.kd * derivative;
    }

    // 生成PWM控制命令
    airsim_ros::RotorPWM generatePWMCommand()
    {
        airsim_ros::RotorPWM pwm_cmd;
        pwm_cmd.header.stamp = ros::Time::now();

        // 初始化为悬停PWM值
        pwm_cmd.rotorPWM0 = hover_pwm_;
        pwm_cmd.rotorPWM1 = hover_pwm_;
        pwm_cmd.rotorPWM2 = hover_pwm_;
        pwm_cmd.rotorPWM3 = hover_pwm_;

        if (!has_path_ || !has_pose_ || !has_imu_ || current_path_.poses.empty())
        {
            return pwm_cmd;
        }

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        if (dt <= 0)
            dt = 0.01; // 防止除零

        // 找到前瞻点
        int target_idx = findLookaheadPoint();
        geometry_msgs::Point target = current_path_.poses[target_idx].pose.position;

        // 计算位置误差
        Eigen::Vector3d pos_error(
            target.x - current_pose_.pose.position.x,
            target.y - current_pose_.pose.position.y,
            target.z - current_pose_.pose.position.z);

        // 计算偏航角误差
        double target_yaw = std::atan2(pos_error.y(), pos_error.x());
        double current_yaw = tf::getYaw(current_pose_.pose.orientation);
        double yaw_error = target_yaw - current_yaw;
        if (yaw_error > M_PI)
            yaw_error -= 2 * M_PI;
        if (yaw_error < -M_PI)
            yaw_error += 2 * M_PI;

        // 计算XY平面控制输出
        Eigen::Vector3d xy_control = computePIDControl(
            Eigen::Vector3d(pos_error.x(), pos_error.y(), 0),
            pos_integral_,
            last_pos_error_,
            xy_gains_,
            dt);

        // 计算Z轴控制输出
        double z_control = computePIDControl(
                               Eigen::Vector3d(0, 0, pos_error.z()),
                               pos_integral_,
                               last_pos_error_,
                               z_gains_,
                               dt)
                               .z();

        // 计算偏航控制输出
        yaw_integral_ += yaw_error * dt;
        double yaw_derivative = (yaw_error - last_yaw_error_) / dt;
        double yaw_control = yaw_gains_.kp * yaw_error +
                             yaw_gains_.ki * yaw_integral_ +
                             yaw_gains_.kd * yaw_derivative;

        // 更新误差记录
        last_pos_error_ = pos_error;
        last_yaw_error_ = yaw_error;
        last_time_ = current_time;

        // 将控制输出转换为PWM值
        double thrust = hover_pwm_ + z_control;
        double pitch = xy_control.x();
        double roll = xy_control.y();

        // PWM分配
        // 右前(0): +pitch +roll -yaw
        pwm_cmd.rotorPWM0 = thrust + pitch + roll - yaw_control;
        // 左后(1): -pitch -roll -yaw
        pwm_cmd.rotorPWM1 = thrust - pitch - roll - yaw_control;
        // 左前(2): -pitch +roll +yaw
        pwm_cmd.rotorPWM2 = thrust - pitch + roll + yaw_control;
        // 右后(3): +pitch -roll +yaw
        pwm_cmd.rotorPWM3 = thrust + pitch - roll + yaw_control;

        // 限制PWM值在有效范围内
        pwm_cmd.rotorPWM0 = std::max(min_pwm_, std::min(max_pwm_, pwm_cmd.rotorPWM0));
        pwm_cmd.rotorPWM1 = std::max(min_pwm_, std::min(max_pwm_, pwm_cmd.rotorPWM1));
        pwm_cmd.rotorPWM2 = std::max(min_pwm_, std::min(max_pwm_, pwm_cmd.rotorPWM2));
        pwm_cmd.rotorPWM3 = std::max(min_pwm_, std::min(max_pwm_, pwm_cmd.rotorPWM3));

        return pwm_cmd;
    }

    void run()
    {
        ros::Rate rate(100); // 100Hz控制频率
        while (ros::ok())
        {
            if (has_path_ && has_pose_ && has_imu_)
            {
                airsim_ros::RotorPWM pwm_cmd = generatePWMCommand();
                pwm_pub_.publish(pwm_cmd);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_controller");
    MotionController controller;
    controller.run();
    return 0;
}