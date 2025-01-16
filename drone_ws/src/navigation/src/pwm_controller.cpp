#include <ros/ros.h>
#include <airsim_ros/RotorPWM.h>
#include <geometry_msgs/PoseStamped.h>

class PWMController
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pwm_pub_;
    ros::Subscriber pose_sub_;

    // PID parameters
    struct PIDParams
    {
        double kp = 0.5;  // Reduced proportional gain
        double ki = 0.01; // Small integral gain
        double kd = 0.3;  // Increased derivative gain
    } pid_params_;

    // Motor PWM values
    struct MotorPWM
    {
        double motor0 = 0.0; // Right front
        double motor1 = 0.0; // Left rear
        double motor2 = 0.0; // Left front
        double motor3 = 0.0; // Right rear
    } motor_pwm_;

    // Error terms
    double prev_error_[3] = {0, 0, 0};
    double integral_error_[3] = {0, 0, 0};

    // Control parameters
    double dt_;
    double scaling_factor_;
    double base_pwm_;
    double max_pwm_;
    double min_pwm_;
    double target_height_;
    double take_off_pwm_;
    bool is_base_pwm_calibrated_;
    double current_base_pwm_;

    void hover_(const double &current_height, airsim_ros::RotorPWM &pwm_msg)
    {
        // Calculate height error
        double error = target_height_ - current_height;

        // 自适应基准PWM
        if (abs(error) < 0.05 && abs(prev_error_[2]) < 0.05) // 如果高度稳定
        {
            current_base_pwm_ = motor_pwm_.motor0; // 使用当前PWM作为新的基准值
            ROS_INFO("Updated base PWM to: %.6f", current_base_pwm_);
        }

        // PID control
        double p_term = pid_params_.kp * error;

        // 积分项处理
        const double max_integral = 0.1; // 减小积分限幅
        if (abs(error) < 0.2)            // 只在误差较小时启用积分
        {
            integral_error_[2] = std::max(-max_integral,
                                          std::min(max_integral, integral_error_[2] + error * dt_));
        }
        else
        {
            integral_error_[2] = 0; // 误差过大时重置积分
        }
        double i_term = pid_params_.ki * integral_error_[2];

        // 微分项处理
        double error_rate = (error - prev_error_[2]) / dt_;
        double d_term = pid_params_.kd * error_rate;
        prev_error_[2] = error;

        // Calculate control output with scaling factor
        double control_output = current_base_pwm_ + (p_term + i_term + d_term) * scaling_factor_;

        ROS_INFO("Height: %.3f, Error: %.3f", current_height, error);
        ROS_INFO("PID terms - P: %.6f, I: %.6f, D: %.6f", p_term, i_term, d_term);
        ROS_INFO("Base PWM: %.6f, Control output: %.6f", current_base_pwm_, control_output);

        // Constrain PWM values
        control_output = std::max(min_pwm_, std::min(max_pwm_, control_output));

        // Apply same PWM to all motors for vertical control
        motor_pwm_.motor0 = control_output;
        motor_pwm_.motor1 = control_output;
        motor_pwm_.motor2 = control_output;
        motor_pwm_.motor3 = control_output;

        // Set PWM values to message
        pwm_msg.header.stamp = ros::Time::now();
        pwm_msg.rotorPWM0 = motor_pwm_.motor0;
        pwm_msg.rotorPWM1 = motor_pwm_.motor1;
        pwm_msg.rotorPWM2 = motor_pwm_.motor2;
        pwm_msg.rotorPWM3 = motor_pwm_.motor3;
    }

public:
    PWMController() : is_base_pwm_calibrated_(false)
    {
        // Load PID parameters
        nh_.param("pwm_controller/kp", pid_params_.kp, 0.5);
        nh_.param("pwm_controller/ki", pid_params_.ki, 0.01);
        nh_.param("pwm_controller/kd", pid_params_.kd, 0.3);

        // Load control parameters
        nh_.param("pwm_controller/dt", dt_, 0.1);
        nh_.param("pwm_controller/scaling_factor", scaling_factor_, 0.000003);
        nh_.param("pwm_controller/base_pwm", base_pwm_, 0.17808237671852112);
        nh_.param("pwm_controller/max_pwm", max_pwm_, 0.5);
        nh_.param("pwm_controller/min_pwm", min_pwm_, 0.0);
        nh_.param("pwm_controller/target_height", target_height_, 1.2);
        nh_.param("pwm_controller/take_off_pwm", take_off_pwm_, 0.33);
        // Log all parameters
        ROS_INFO("PWM Controller Parameters loaded:");
        ROS_INFO("PID Parameters - kp: %.3f, ki: %.3f, kd: %.3f", pid_params_.kp, pid_params_.ki, pid_params_.kd);
        ROS_INFO("Control Parameters:");
        ROS_INFO("dt: %.3f", dt_);
        ROS_INFO("scaling_factor: %.6f", scaling_factor_);
        ROS_INFO("base_pwm: %.3f", base_pwm_);
        ROS_INFO("max_pwm: %.3f", max_pwm_);
        ROS_INFO("min_pwm: %.3f", min_pwm_);
        ROS_INFO("take_off_pwm: %.3f", take_off_pwm_);
        ROS_INFO("target_height: %.3f", target_height_);

        // Initialize publishers and subscribers
        pwm_pub_ = nh_.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 100);
        pose_sub_ = nh_.subscribe("/airsim_node/drone_1/drone_pose", 10, &PWMController::poseCallback, this);

        // Initialize error terms
        for (int i = 0; i < 3; i++)
        {
            prev_error_[i] = 0.0;
            integral_error_[i] = 0.0;
        }

        // 初始基准PWM
        current_base_pwm_ = base_pwm_;

        ROS_INFO("Initial base PWM: %.6f", current_base_pwm_);

        ROS_INFO("PWM Controller initialized");
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // Get current height
        double current_height = msg->pose.position.z;

        // Create PWM message
        airsim_ros::RotorPWM pwm_msg;

        // Call hover control
        hover_(current_height, pwm_msg);

        // Publish PWM values
        pwm_pub_.publish(pwm_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pwm_controller");
    PWMController controller;
    ros::spin();
    return 0;
}
