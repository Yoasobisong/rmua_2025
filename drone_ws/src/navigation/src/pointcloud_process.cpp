#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <visualization_msgs/MarkerArray.h>

class PointCloudProcessor
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher ground_removed_pub_;
    ros::Publisher obstacles_pub_;
    ros::Publisher marker_pub_;

    // PCL滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;
    pcl::SACSegmentation<pcl::PointXYZ> ground_seg_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction_;

    // 参数
    double voxel_leaf_size_;
    int mean_k_;
    double std_dev_mul_thresh_;
    double ground_distance_thresh_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

public:
    PointCloudProcessor() : nh_("~")
    {
        // 加载参数
        nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.1);
        nh_.param("mean_k", mean_k_, 30);
        nh_.param("std_dev_mul_thresh", std_dev_mul_thresh_, 1.5);
        nh_.param("ground_distance_thresh", ground_distance_thresh_, 0.3);
        nh_.param("cluster_tolerance", cluster_tolerance_, 0.8);
        nh_.param("min_cluster_size", min_cluster_size_, 50);
        nh_.param("max_cluster_size", max_cluster_size_, 50000);

        // 设置订阅者和发布者
        cloud_sub_ = nh_.subscribe("/airsim_node/drone_1/lidar", 1,
                                   &PointCloudProcessor::cloudCallback, this);
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/airsim_node/drone_1//pointcloud/filtered", 1);
        ground_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/airsim_node/drone_1//pointcloud/ground_removed", 1);
        obstacles_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/airsim_node/drone_1//pointcloud/obstacles", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/airsim_node/drone_1//obstacles/markers", 1);

        // 配置地面分割器
        ground_seg_.setOptimizeCoefficients(true);
        ground_seg_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        ground_seg_.setMethodType(pcl::SAC_RANSAC);
        ground_seg_.setDistanceThreshold(ground_distance_thresh_);
        ground_seg_.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
        ground_seg_.setEpsAngle(20.0 * (M_PI / 180.0));

        // 配置聚类提取器
        cluster_extraction_.setClusterTolerance(cluster_tolerance_);
        cluster_extraction_.setMinClusterSize(min_cluster_size_);
        cluster_extraction_.setMaxClusterSize(max_cluster_size_);

        // ROS_INFO("PointCloud Processor initialized with parameters:");
        // ROS_INFO("Voxel leaf size: %.3f", voxel_leaf_size_);
        // ROS_INFO("Mean K: %d", mean_k_);
        // ROS_INFO("Std dev multiplier: %.3f", std_dev_mul_thresh_);
        // ROS_INFO("Ground distance threshold: %.3f", ground_distance_thresh_);
        // ROS_INFO("Cluster tolerance: %.3f", cluster_tolerance_);
        // ROS_INFO("Min cluster size: %d", min_cluster_size_);
        // ROS_INFO("Max cluster size: %d", max_cluster_size_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
    {
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        // ROS_INFO("Received cloud with %lu points", cloud->size());

        // 体素滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter_.filter(*cloud_voxel_filtered);
        // ROS_INFO("After voxel filtering: %lu points", cloud_voxel_filtered->size());

        // 统计滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        outlier_filter_.setInputCloud(cloud_voxel_filtered);
        outlier_filter_.setMeanK(mean_k_);
        outlier_filter_.setStddevMulThresh(std_dev_mul_thresh_);
        outlier_filter_.filter(*cloud_filtered);
        // ROS_INFO("After statistical filtering: %lu points", cloud_filtered->size());

        // 地面分割
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        ground_seg_.setInputCloud(cloud_filtered);
        ground_seg_.segment(*inliers, *coefficients);
        // ROS_INFO("Ground plane coefficients: %.3f, %.3f, %.3f, %.3f",
        //          coefficients->values[0], coefficients->values[1],
        //          coefficients->values[2], coefficients->values[3]);

        // 提取非地面点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_no_ground);
        // ROS_INFO("After ground removal: %lu points", cloud_no_ground->size());

        // 创建KdTree用于聚类搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_no_ground);

        // 执行欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        cluster_extraction_.setSearchMethod(tree);
        cluster_extraction_.setInputCloud(cloud_no_ground);
        cluster_extraction_.extract(cluster_indices);
        // ROS_INFO("Found %lu clusters", cluster_indices.size());

        // 为每个聚类创建一个新的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        for (const auto &indices : cluster_indices)
        {
            // 提取当前聚类的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &index : indices.indices)
            {
                cluster->points.push_back(cloud_no_ground->points[index]);
                obstacles->points.push_back(cloud_no_ground->points[index]);
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;

            // 计算聚类的边界框
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            // 创建边界框标记
            visualization_msgs::Marker marker;
            marker.header = cloud_msg->header;
            marker.ns = "obstacles";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // 设置边界框的位置和大小
            marker.pose.position.x = (min_pt.x + max_pt.x) / 2;
            marker.pose.position.y = (min_pt.y + max_pt.y) / 2;
            marker.pose.position.z = (min_pt.z + max_pt.z) / 2;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = std::max(max_pt.x - min_pt.x, 0.1f);
            marker.scale.y = std::max(max_pt.y - min_pt.y, 0.1f);
            marker.scale.z = std::max(max_pt.z - min_pt.z, 0.1f);

            // 设置边界框为白色半透明
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.3; // 降低透明度使其更透明

            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);
        }

        // 发布处理后的点云和标记
        sensor_msgs::PointCloud2 filtered_msg, no_ground_msg, obstacles_msg;
        pcl::toROSMsg(*cloud_filtered, filtered_msg);
        pcl::toROSMsg(*cloud_no_ground, no_ground_msg);
        pcl::toROSMsg(*obstacles, obstacles_msg);

        filtered_msg.header = cloud_msg->header;
        no_ground_msg.header = cloud_msg->header;
        obstacles_msg.header = cloud_msg->header;

        filtered_cloud_pub_.publish(filtered_msg);
        ground_removed_pub_.publish(no_ground_msg);
        obstacles_pub_.publish(obstacles_msg);
        marker_pub_.publish(marker_array);

        // ROS_INFO("Published all messages");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_process");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}