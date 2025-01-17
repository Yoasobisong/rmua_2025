#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

class PointCloudProcessor
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_cloud_pub_;

    // PCL滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_;

    // 参数
    double voxel_leaf_size_;
    int mean_k_;
    double std_dev_mul_thresh_;
    double y_min_, y_max_;
    double max_time_diff_;     // 最大时间差
    int min_points_threshold_; // 最小点数阈值

    // 时间平滑相关
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_filtered_cloud_;
    ros::Time last_update_time_;

public:
    PointCloudProcessor() : nh_("~"),
                            last_filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // Load parameters
        nh_.param("voxel_filter/leaf_size", voxel_leaf_size_, 0.15);
        nh_.param("statistical_filter/mean_k", mean_k_, 30);
        nh_.param("statistical_filter/std_dev_mul_thresh", std_dev_mul_thresh_, 1.0);
        nh_.param("filter/y_min", y_min_, -7.0);
        nh_.param("filter/y_max", y_max_, 7.0);
        nh_.param("filter/max_time_diff", max_time_diff_, 0.5);               // 最大时间差0.5秒
        nh_.param("filter/min_points_threshold", min_points_threshold_, 100); // 最小点数阈值

        // 设置订阅者和发布者
        cloud_sub_ = nh_.subscribe("/airsim_node/drone_1/lidar", 1,
                                   &PointCloudProcessor::cloudCallback, this);
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/airsim_node/drone_1/pointcloud/filtered", 1);

        // 配置Y轴过滤器
        pass_through_.setFilterFieldName("y");
        pass_through_.setFilterLimits(y_min_, y_max_);

        last_update_time_ = ros::Time::now();
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
    {
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 体素滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter_.filter(*cloud_voxel_filtered);

        // Y轴范围限制
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass_through_.setInputCloud(cloud_voxel_filtered);
        pass_through_.filter(*cloud_y_filtered);

        // 统计滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        outlier_filter_.setInputCloud(cloud_y_filtered);
        outlier_filter_.setMeanK(mean_k_);
        outlier_filter_.setStddevMulThresh(std_dev_mul_thresh_);
        outlier_filter_.filter(*cloud_filtered);

        // 检查点云是否有效
        ros::Time current_time = ros::Time::now();
        double time_diff = (current_time - last_update_time_).toSec();

        // 如果当前点云点数太少且时间间隔不大，使用上一帧的点云
        if (cloud_filtered->points.size() < min_points_threshold_ &&
            time_diff < max_time_diff_ &&
            !last_filtered_cloud_->points.empty())
        {
            cloud_filtered = last_filtered_cloud_;
            ROS_WARN_THROTTLE(1.0, "Using last frame point cloud due to insufficient points (current: %lu, threshold: %d)",
                              cloud_filtered->points.size(), min_points_threshold_);
        }
        else
        {
            // 更新上一帧的点云和时间
            last_filtered_cloud_ = cloud_filtered;
            last_update_time_ = current_time;
        }

        // 发布处理后的点云
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered, filtered_msg);
        filtered_msg.header = cloud_msg->header;
        filtered_cloud_pub_.publish(filtered_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_process");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}