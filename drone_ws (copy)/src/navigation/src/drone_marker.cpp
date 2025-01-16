#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

class DroneMarker
{
private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber pose_sub_;
    visualization_msgs::Marker drone_marker_;

    // Drone size parameters (in meters)
    const double DRONE_LENGTH = 0.5; // Length
    const double DRONE_WIDTH = 0.5;  // Width
    const double DRONE_HEIGHT = 0.2; // Height

    void initDroneMarker()
    {
        drone_marker_.header.frame_id = "drone_init";
        drone_marker_.ns = "drone_body";
        drone_marker_.id = 0;
        drone_marker_.type = visualization_msgs::Marker::CUBE;
        drone_marker_.action = visualization_msgs::Marker::ADD;

        // Set drone size
        drone_marker_.scale.x = DRONE_LENGTH;
        drone_marker_.scale.y = DRONE_WIDTH;
        drone_marker_.scale.z = DRONE_HEIGHT;

        // Set color (semi-transparent white)
        drone_marker_.color.r = 1.0;
        drone_marker_.color.g = 1.0;
        drone_marker_.color.b = 1.0;
        drone_marker_.color.a = 0.5;

        // Set position (relative to drone_init center)
        drone_marker_.pose.position.x = 0;
        drone_marker_.pose.position.y = 0;
        drone_marker_.pose.position.z = 0;

        // Set lifetime (0 means forever)
        drone_marker_.lifetime = ros::Duration(0);
    }

    // Function to invert quaternion
    geometry_msgs::Quaternion invertQuaternion(const geometry_msgs::Quaternion &q)
    {
        tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
        tf_quat = tf_quat.inverse();
        geometry_msgs::Quaternion inverted_q;
        tf::quaternionTFToMsg(tf_quat, inverted_q);
        return inverted_q;
    }

public:
    DroneMarker() : nh_("~")
    {
        // Set up publisher and subscriber
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/airsim_node/drone_1/drone_marker", 1);

        // Use global namespace for subscriber
        ros::NodeHandle nh;
        pose_sub_ = nh.subscribe("/airsim_node/drone_1/drone_pose", 1, &DroneMarker::poseCallback, this);

        // Initialize drone marker
        initDroneMarker();
        ROS_INFO("Drone marker node initialized");
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        drone_marker_.header.stamp = ros::Time::now();

        // Update position from pose message
        drone_marker_.pose.position = msg->pose.position;

        // Update orientation with inverted quaternion
        drone_marker_.pose.orientation = invertQuaternion(msg->pose.orientation);

        marker_pub_.publish(drone_marker_);
    }

    void run()
    {
        ros::Rate rate(20); // 20Hz for smooth visualization
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_marker");
    DroneMarker drone_marker;
    drone_marker.run();
    return 0;
}