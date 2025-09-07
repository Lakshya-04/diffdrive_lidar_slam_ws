#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL filters - COMPLETE INCLUDES
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>      // MISSING - This was the main issue!
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>                // MISSING - For clustering
#include <pcl/common/common.h>                // MISSING - For basic operations
#include <pcl/filters/filter.h>               // MISSING - For removeNaNFromPointCloud

class PCLProcessor : public rclcpp::Node
{
public:
    PCLProcessor() : Node("pcl_processor")
    {
        // Subscribers for different LiDAR types
        lidar_2d_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan_cloud", 10, std::bind(&PCLProcessor::process2DLidar, this, std::placeholders::_1));
        
        lidar_3d_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_3d", 10, std::bind(&PCLProcessor::process3DLidar, this, std::placeholders::_1));
        
        rgbd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10, std::bind(&PCLProcessor::processRGBD, this, std::placeholders::_1));

        // Publishers for filtered clouds
        filtered_2d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_filtered", 10);
        filtered_3d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_3d_filtered", 10);
        obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles", 10);
        ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_plane", 10);

        RCLCPP_INFO(this->get_logger(), "PCL Processor initialized");
    }

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_2d_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_3d_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rgbd_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_2d_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_3d_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;

    void process2DLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Convert ROS message to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);

            if (cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty 2D LiDAR cloud");
                return;
            }

            // 2D LiDAR processing pipeline
            auto filtered_cloud = apply2DFilters(cloud);

            // Convert back to ROS message and publish
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*filtered_cloud, output);
            output.header = msg->header;
            filtered_2d_pub_->publish(output);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing 2D LiDAR: %s", e.what());
        }
    }

    void process3DLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Convert ROS message to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);

            if (cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty 3D LiDAR cloud");
                return;
            }

            // 3D LiDAR processing pipeline
            auto results = apply3DFilters(cloud);

            // Publish different outputs
            publishFilteredCloud(results.filtered, filtered_3d_pub_, msg->header);
            publishFilteredCloud(results.obstacles, obstacles_pub_, msg->header);
            publishFilteredCloud(results.ground, ground_pub_, msg->header);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing 3D LiDAR: %s", e.what());
        }
    }

    void processRGBD(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Convert ROS message to PCL with RGB
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*msg, *cloud);

            if (cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty RGBD cloud");
                return;
            }

            // RGBD-specific processing
            auto filtered_cloud = applyRGBDFilters(cloud);
            
            // Publish result
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*filtered_cloud, output);
            output.header = msg->header;
            filtered_3d_pub_->publish(output);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGBD: %s", e.what());
        }
    }

    // 2D LIDAR FILTERS
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply2DFilters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 1. PassThrough Filter - Remove points outside working area
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-10.0, 10.0);  // 20m working radius
        pass.filter(*filtered);

        pass.setInputCloud(filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-10.0, 10.0);
        pass.filter(*filtered);

        // 2. Statistical Outlier Removal - Remove noise
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered);
        sor.setMeanK(20);                // Analyze 20 neighbors
        sor.setStddevMulThresh(1.0);     // Conservative threshold
        sor.filter(*filtered);

        return filtered;
    }

    // 3D LIDAR FILTERS
    struct FilterResults {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
    };

    FilterResults apply3DFilters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        FilterResults results;
        results.filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        results.obstacles = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        results.ground = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        // 1. Voxel Grid Downsampling - Performance optimization
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);  // 5cm resolution
        vg.filter(*downsampled);

        // 2. Working Area Filter - Remove irrelevant points
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(-15.0, -15.0, -1.0, 1.0));  // AGV working area
        boxFilter.setMax(Eigen::Vector4f(15.0, 15.0, 3.0, 1.0));
        boxFilter.setInputCloud(downsampled);
        boxFilter.filter(*results.filtered);

        // 3. Ground Plane Segmentation - Critical for navigation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.1);    // 10cm threshold for ground plane

        seg.setInputCloud(results.filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "Could not find ground plane in point cloud");
            *results.obstacles = *results.filtered;  // Treat all points as obstacles
            return results;
        }

        // Extract ground and obstacles - FIXED WITH PROPER INCLUDES
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(results.filtered);
        extract.setIndices(inliers);
        
        // Ground points
        extract.setNegative(false);
        extract.filter(*results.ground);
        
        // Obstacle points (non-ground)
        extract.setNegative(true);
        extract.filter(*results.obstacles);

        // 4. Cluster Extraction for obstacles (optional)
        clusterObstacles(results.obstacles);

        return results;
    }

    // RGBD-specific filters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyRGBDFilters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Remove NaN points common in RGBD
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);

        // PassThrough for depth range
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.3, 5.0);  // Useful depth range
        pass.filter(*filtered);

        return filtered;
    }

    void clusterObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles)
    {
        if (obstacles->empty()) return;

        // Euclidean clustering for obstacle detection
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(obstacles);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.3);   // 30cm
        ec.setMinClusterSize(10);      // Minimum points per cluster
        ec.setMaxClusterSize(1000);    // Maximum points per cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(obstacles);
        ec.extract(cluster_indices);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            "Detected %zu obstacle clusters", cluster_indices.size());
    }

    void publishFilteredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                             rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                             const std_msgs::msg::Header& header)
    {
        if (!cloud->empty()) {
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header = header;
            pub->publish(output);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLProcessor>());
    rclcpp::shutdown();
    return 0;
}