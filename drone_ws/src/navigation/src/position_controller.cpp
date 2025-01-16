#include <ros/ros.h>
#include <airsim_ros/RotorPWM.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_srvs/Empty.h>

class PositionController
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pwm_pub_;
    ros::Subscriber pose_sub_;
    ros::ServiceServer start_service_;
    bool is_started_ = false; // Flag to control when to start flying

    // Target position
    struct TargetPosition
    {
        double x = 2.0;
        double y = 1.0;
        double z = 1.5;
    } target_pos_;

    // PID parameters for each axis
    struct PIDParams
    {
        double kp = 2.0;
        double ki = 0.1;
        double kd = 1.0;
    } pid_params_x_, pid_params_y_;

    struct PIDParamsZ
    {
        double kp = 3.0;
        double ki = 0.2;
        double kd = 1.5;
    } pid_params_z_;

    // Motor PWM values
    struct MotorPWM
    {
        double motor0 = 0.0; // Right front
        double motor1 = 0.0; // Left rear
        double motor2 = 0.0; // Left front
        double motor3 = 0.0; // Right rear
    } motor_pwm_;

    // Error terms for each axis
    double prev_error_[3] = {0, 0, 0}; // x, y, z
    double integral_error_[3] = {0, 0, 0};

    // Control parameters
    double dt_;
    double scaling_factor_;
    double base_pwm_;
    double max_pwm_;
    double min_pwm_;

    void calculatePWM_(const geometry_msgs::PoseStamped::ConstPtr &msg, airsim_ros::RotorPWM &pwm_msg)
    {
        // Get current position
        double current_x = msg->pose.position.x;
        double current_y = msg->pose.position.y;
        double current_z = msg->pose.position.z;

        // Calculate position errors in world frame
        double error_x = target_pos_.x - current_x;
        double error_y = target_pos_.y - current_y;
        double error_z = target_pos_.z - current_z;

        // Get current orientation
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Transform position errors from world frame to body frame
        double error_x_body = error_x * cos(yaw) + error_y * sin(yaw);
        double error_y_body = -error_x * sin(yaw) + error_y * cos(yaw);

        // Integral anti-windup with reduced limit
        const double max_integral = 0.3;

        // PID control for X axis (body frame)
        double p_term_x = pid_params_x_.kp * error_x_body;
        integral_error_[0] = std::max(-max_integral, std::min(max_integral, integral_error_[0] + error_x_body * dt_));
        double i_term_x = pid_params_x_.ki * integral_error_[0];
        double d_term_x = pid_params_x_.kd * (error_x_body - prev_error_[0]) / dt_;
        prev_error_[0] = error_x_body;
        double control_x = p_term_x + i_term_x + d_term_x;

        // PID control for Y axis (body frame)
        double p_term_y = pid_params_y_.kp * error_y_body;
        integral_error_[1] = std::max(-max_integral, std::min(max_integral, integral_error_[1] + error_y_body * dt_));
        double i_term_y = pid_params_y_.ki * integral_error_[1];
        double d_term_y = pid_params_y_.kd * (error_y_body - prev_error_[1]) / dt_;
        prev_error_[1] = error_y_body;
        double control_y = p_term_y + i_term_y + d_term_y;

        // PID control for Z axis with increased control effect
        double p_term_z = pid_params_z_.kp * error_z;
        integral_error_[2] = std::max(-max_integral, std::min(max_integral, integral_error_[2] + error_z * dt_));
        double i_term_z = pid_params_z_.ki * integral_error_[2];
        double d_term_z = pid_params_z_.kd * (error_z - prev_error_[2]) / dt_;
        prev_error_[2] = error_z;
        double control_z = p_term_z + i_term_z + d_term_z;

        // Calculate distance to target
        double distance = sqrt(error_x * error_x + error_y * error_y);

        // Dynamic max tilt angle based on distance to target
        const double max_tilt_far = 0.15;      // About 8.6 degrees when far
        const double max_tilt_near = 0.08;     // About 4.6 degrees when near
        const double distance_threshold = 2.0; // Distance to start reducing tilt
        double max_tilt;

        if (distance > distance_threshold)
        {
            max_tilt = max_tilt_far;
        }
        else
        {
            // Linearly reduce max tilt as we get closer
            max_tilt = max_tilt_near + (max_tilt_far - max_tilt_near) * (distance / distance_threshold);
        }

        // Calculate thrust with height control
        double thrust = base_pwm_;
        if (error_z > 0)
        {
            thrust += 0.001; // Small increment for positive errors
        }
        thrust = std::max(min_pwm_, std::min(max_pwm_, thrust));

        // Limit maximum control outputs based on distance
        double max_control = distance > distance_threshold ? 1.0 : 0.5;
        control_x = std::max(-max_control, std::min(max_control, control_x));
        control_y = std::max(-max_control, std::min(max_control, control_y));

        // Calculate attitude control with dynamic limits
        double roll_pwm = control_y * scaling_factor_;
        double pitch_pwm = -control_x * scaling_factor_;

        // Limit roll and pitch PWM
        roll_pwm = std::max(-max_tilt * scaling_factor_, std::min(max_tilt * scaling_factor_, roll_pwm));
        pitch_pwm = std::max(-max_tilt * scaling_factor_, std::min(max_tilt * scaling_factor_, pitch_pwm));

        // Add stronger damping when close to target
        double damping = distance > distance_threshold ? 0.0001 : 0.0002;
        roll_pwm -= damping * roll;
        pitch_pwm -= damping * pitch;

        // Calculate motor PWM values
        motor_pwm_.motor0 = thrust + roll_pwm + pitch_pwm; // Right front
        motor_pwm_.motor1 = thrust - roll_pwm - pitch_pwm; // Left rear
        motor_pwm_.motor2 = thrust - roll_pwm + pitch_pwm; // Left front
        motor_pwm_.motor3 = thrust + roll_pwm - pitch_pwm; // Right rear

        // Add attitude stabilization with dynamic threshold
        double tilt_threshold = distance > distance_threshold ? 0.2 : 0.1; // Reduce threshold near target
        if (fabs(roll) > tilt_threshold || fabs(pitch) > tilt_threshold)
        {
            // Calculate stabilization factor based on distance
            double stabilize_factor = distance > distance_threshold ? 0.0001 : 0.0002;
            double stabilize = stabilize_factor * (fabs(roll) + fabs(pitch));

            // Apply stabilization more aggressively when close to target
            if (roll > 0)
            {
                motor_pwm_.motor1 += stabilize;
                motor_pwm_.motor2 += stabilize;
            }
            else
            {
                motor_pwm_.motor0 += stabilize;
                motor_pwm_.motor3 += stabilize;
            }
            if (pitch > 0)
            {
                motor_pwm_.motor1 += stabilize;
                motor_pwm_.motor3 += stabilize;
            }
            else
            {
                motor_pwm_.motor0 += stabilize;
                motor_pwm_.motor2 += stabilize;
            }
        }

        // Add extra stability near target
        if (distance < 0.5)
        { // Very close to target
            double hover_stabilize = 0.0001;
            motor_pwm_.motor0 += hover_stabilize;
            motor_pwm_.motor1 += hover_stabilize;
            motor_pwm_.motor2 += hover_stabilize;
            motor_pwm_.motor3 += hover_stabilize;
        }

        // Constrain PWM values
        motor_pwm_.motor0 = std::max(min_pwm_, std::min(max_pwm_, motor_pwm_.motor0));
        motor_pwm_.motor1 = std::max(min_pwm_, std::min(max_pwm_, motor_pwm_.motor1));
        motor_pwm_.motor2 = std::max(min_pwm_, std::min(max_pwm_, motor_pwm_.motor2));
        motor_pwm_.motor3 = std::max(min_pwm_, std::min(max_pwm_, motor_pwm_.motor3));

        // Log control information
        ROS_INFO("Current position - X: %.3f, Y: %.3f, Z: %.3f", current_x, current_y, current_z);
        ROS_INFO("Current attitude - Roll: %.3f, Pitch: %.3f, Yaw: %.3f", roll, pitch, yaw);
        ROS_INFO("Position error (world) - X: %.3f, Y: %.3f, Z: %.3f", error_x, error_y, error_z);
        ROS_INFO("Position error (body) - X: %.3f, Y: %.3f", error_x_body, error_y_body);
        ROS_INFO("Control output - X: %.6f, Y: %.6f, Z: %.6f", control_x, control_y, control_z);
        ROS_INFO("PWM values - M0: %.6f, M1: %.6f, M2: %.6f, M3: %.6f",
                 motor_pwm_.motor0, motor_pwm_.motor1, motor_pwm_.motor2, motor_pwm_.motor3);

        // Set PWM message
        pwm_msg.header.stamp = ros::Time::now();
        pwm_msg.rotorPWM0 = motor_pwm_.motor0;
        pwm_msg.rotorPWM1 = motor_pwm_.motor1;
        pwm_msg.rotorPWM2 = motor_pwm_.motor2;
        pwm_msg.rotorPWM3 = motor_pwm_.motor3;
    }

    bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        is_started_ = true;
        ROS_INFO("Position controller received start command!");
        return true;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!is_started_) // Only control if started
            return;

        airsim_ros::RotorPWM pwm_msg;
        calculatePWM_(msg, pwm_msg);
        pwm_pub_.publish(pwm_msg);
    }

public:
    PositionController()
    {
        // Load PID parameters for each axis
        nh_.param("position_controller/x/kp", pid_params_x_.kp, 2.0);
        nh_.param("position_controller/x/ki", pid_params_x_.ki, 0.1);
        nh_.param("position_controller/x/kd", pid_params_x_.kd, 1.0);

        nh_.param("position_controller/y/kp", pid_params_y_.kp, 2.0);
        nh_.param("position_controller/y/ki", pid_params_y_.ki, 0.1);
        nh_.param("position_controller/y/kd", pid_params_y_.kd, 1.0);

        nh_.param("position_controller/z/kp", pid_params_z_.kp, 3.0);
        nh_.param("position_controller/z/ki", pid_params_z_.ki, 0.2);
        nh_.param("position_controller/z/kd", pid_params_z_.kd, 1.5);

        // Load control parameters
        nh_.param("position_controller/dt", dt_, 0.1);
        nh_.param("position_controller/scaling_factor", scaling_factor_, 0.001);
        nh_.param("position_controller/base_pwm", base_pwm_, 0.3);
        nh_.param("position_controller/max_pwm", max_pwm_, 0.8);
        nh_.param("position_controller/min_pwm", min_pwm_, 0.0);

        // Log parameters
        ROS_INFO("Position Controller Parameters:");
        ROS_INFO("X axis - kp: %.3f, ki: %.3f, kd: %.3f",
                 pid_params_x_.kp, pid_params_x_.ki, pid_params_x_.kd);
        ROS_INFO("Y axis - kp: %.3f, ki: %.3f, kd: %.3f",
                 pid_params_y_.kp, pid_params_y_.ki, pid_params_y_.kd);
        ROS_INFO("Z axis - kp: %.3f, ki: %.3f, kd: %.3f",
                 pid_params_z_.kp, pid_params_z_.ki, pid_params_z_.kd);
        ROS_INFO("Control Parameters:");
        ROS_INFO("dt: %.3f", dt_);
        ROS_INFO("scaling_factor: %.6f", scaling_factor_);
        ROS_INFO("base_pwm: %.6f", base_pwm_);
        ROS_INFO("max_pwm: %.3f", max_pwm_);
        ROS_INFO("min_pwm: %.3f", min_pwm_);
        ROS_INFO("Target position - X: %.3f, Y: %.3f, Z: %.3f",
                 target_pos_.x, target_pos_.y, target_pos_.z);

        // Initialize publishers and subscribers
        pwm_pub_ = nh_.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 100);
        pose_sub_ = nh_.subscribe("/airsim_node/drone_1/drone_pose", 10,
                                  &PositionController::poseCallback, this);
        start_service_ = nh_.advertiseService("/drone_1/start_flight", &PositionController::startCallback, this);

        ROS_INFO("Position controller initialized. Waiting for start command...");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_controller");
    PositionController controller;
    ros::spin();
    return 0;
}