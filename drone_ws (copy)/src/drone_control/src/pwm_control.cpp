#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <airsim_ros/RotorPWM.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32MultiArray.h>

class DroneController
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pwm_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber cmd_vel_sub_; // 接收期望的速度命令

    // PWM控制参数
    static constexpr double BASE_PWM = 0.6;  // 基础PWM值（悬停）
    static constexpr double MAX_PWM = 0.8;   // 最大PWM值
    static constexpr double MIN_PWM = 0.4;   // 最小PWM值
    static constexpr double PWM_SCALE = 0.2; // PWM调整范围

    // PID控制参数
    struct PIDParams
    {
        double kp, ki, kd;
        double integral;
        double prev_error;
        PIDParams() : kp(0), ki(0), kd(0), integral(0), prev_error(0) {}
    };

    PIDParams roll_pid_;  // 横滚角PID
    PIDParams pitch_pid_; // 俯仰角PID
    PIDParams yaw_pid_;   // 偏航角PID

    // 当前姿态
    double current_roll_, current_pitch_, current_yaw_;
    // 目标姿态
    double target_roll_, target_pitch_, target_yaw_;

    ros::Time last_time_;
    bool imu_initialized_;

public:
    DroneController() : imu_initialized_(false)
    {
        // 初始化发布者和订阅者
        pwm_pub_ = nh_.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 10);
        imu_sub_ = nh_.subscribe("/airsim_node/drone_1/imu/imu", 10, &DroneController::imuCallback, this);
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &DroneController::cmdVelCallback, this);

        // 初始化PID参数
        initPIDParams();

        ROS_INFO("Drone PWM controller initialized");
    }

    void initPIDParams()
    {
        // 横滚角PID参数
        roll_pid_.kp = 0.1;
        roll_pid_.ki = 0.01;
        roll_pid_.kd = 0.05;

        // 俯仰角PID参数
        pitch_pid_.kp = 0.1;
        pitch_pid_.ki = 0.01;
        pitch_pid_.kd = 0.05;

        // 偏航角PID参数
        yaw_pid_.kp = 0.15;
        yaw_pid_.ki = 0.0;
        yaw_pid_.kd = 0.1;
    }

    double updatePID(PIDParams &pid, double error, double dt)
    {
        pid.integral += error * dt;
        double derivative = (error - pid.prev_error) / dt;
        pid.prev_error = error;

        return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // 将四元数转换为欧拉角
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 mat(quat);
        mat.getRPY(current_roll_, current_pitch_, current_yaw_);

        if (!imu_initialized_)
        {
            target_roll_ = current_roll_;
            target_pitch_ = current_pitch_;
            target_yaw_ = current_yaw_;
            last_time_ = ros::Time::now();
            imu_initialized_ = true;
            return;
        }

        // 计算时间间隔
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        // 计算姿态误差
        double roll_error = target_roll_ - current_roll_;
        double pitch_error = target_pitch_ - current_pitch_;
        double yaw_error = target_yaw_ - current_yaw_;

        // 使用PID控制器计算控制输出
        double roll_output = updatePID(roll_pid_, roll_error, dt);
        double pitch_output = updatePID(pitch_pid_, pitch_error, dt);
        double yaw_output = updatePID(yaw_pid_, yaw_error, dt);

        // 计算每个电机的PWM值
        // PWM控制(0:右前, 1:左后, 2:左前, 3:右后)
        airsim_ros::RotorPWM pwm_msg;
        pwm_msg.pwm[0] = BASE_PWM + pitch_output - roll_output - yaw_output; // 右前
        pwm_msg.pwm[1] = BASE_PWM - pitch_output + roll_output - yaw_output; // 左后
        pwm_msg.pwm[2] = BASE_PWM - pitch_output - roll_output + yaw_output; // 左前
        pwm_msg.pwm[3] = BASE_PWM + pitch_output + roll_output + yaw_output; // 右后

        // 限制PWM值在合理范围内
        for (int i = 0; i < 4; i++)
        {
            pwm_msg.pwm[i] = std::max(MIN_PWM, std::min(MAX_PWM, pwm_msg.pwm[i]));
        }

        pwm_pub_.publish(pwm_msg);
    }

    void cmdVelCallback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        if (!imu_initialized_)
            return;

        // 将速度命令转换为目标姿态角
        target_pitch_ = -msg->x * 0.2; // 前后运动
        target_roll_ = msg->y * 0.2;   // 左右运动
        target_yaw_ += msg->z * 0.1;   // 偏航角速度
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_pwm_controller");
    DroneController controller;
    ros::spin();
    return 0;
}