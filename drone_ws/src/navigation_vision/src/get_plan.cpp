#include <ros/ros.h>
#include <navigation_vision/VisionYolo.h>
#include <navigation_vision/Box.h>
#include <vector>

class PlanManager
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber vision_sub_;

    // Store detection results
    std::vector<navigation_vision::Box> boxes_; // Array of Box messages
    std::vector<int> classes_;                  // Class IDs
    std::vector<float> confidences_;            // Confidence scores
    std::vector<std::string> class_names_;      // Class names

public:
    PlanManager()
    {
        // Subscribe to YOLO detection results
        vision_sub_ = nh_.subscribe("/airsim_node/drone_1/front_right/vision_result", 1,
                                    &PlanManager::visionCallback, this);
        ROS_INFO("Plan Manager initialized");
    }

    void visionCallback(const navigation_vision::VisionYolo::ConstPtr &msg)
    {
        // Clear previous data
        boxes_.clear();
        classes_.clear();
        confidences_.clear();
        class_names_.clear();

        // Process each detection
        for (size_t i = 0; i < msg->num_detections; ++i)
        {
            // Store box directly
            boxes_.push_back(msg->boxes[i]);

            // Get class information
            classes_.push_back(msg->classes[i]);
            confidences_.push_back(msg->confidences[i]);
            class_names_.push_back(msg->class_names[i]);
        }

        // TODO: Add your planning logic here
        processPlan();
    }

    void processPlan()
    {
        // TODO: Add your planning logic here
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_plan");
    PlanManager manager;
    manager.run();
    return 0;
}
