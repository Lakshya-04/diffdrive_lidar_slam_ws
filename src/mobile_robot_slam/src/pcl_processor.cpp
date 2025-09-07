#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <mutex>
#include <queue>

// PCL includes (same as before)
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>

class AsyncPCLProcessor : public rclcpp::Node
{
public:
    AsyncPCLProcessor() : Node("async_pcl_processor"), processing_(false)
    {
        // Subscribers
        lidar_3d_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_3d", 10, std::bind(&AsyncPCLProcessor::cloudCallback, this, std::placeholders::_1));
        
        // Publishers
        filtered_3d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_3d_filtered", 10);
        obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles", 10);
        ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_plane", 10);

        // Timer for regular publishing at fixed rate (reduces flickering)
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz - smooth visualization
            std::bind(&AsyncPCLProcessor::publishResults, this));

        // Processing thread
        processing_thread_ = std::thread(&AsyncPCLProcessor::processingLoop, this);

        RCLCPP_INFO(this->get_logger(), "Async PCL Processor initialized - 10Hz output");
    }

    ~AsyncPCLProcessor()
    {
        stop_processing_ = true;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

private:
    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_3d_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_3d_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    
    // Threading
    std::thread processing_thread_;
    std::atomic<bool> stop_processing_{false};
    std::atomic<bool> processing_{false};
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // Thread-safe data storage
    std::mutex data_mutex_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue_;
    
    // Results storage
    std::mutex results_mutex_;
    sensor_msgs::msg::PointCloud2 latest_filtered_;
    sensor_msgs::msg::PointCloud2 latest_obstacles_;
    sensor_msgs::msg::PointCloud2 latest_ground_;
    bool has_results_{false};

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Keep only the latest cloud to prevent queue buildup
        while (!cloud_queue_.empty()) {
            cloud_queue_.pop();
        }
        cloud_queue_.push(msg);
    }

    void processingLoop()
    {
        while (!stop_processing_) {
            sensor_msgs::msg::PointCloud2::SharedPtr msg = nullptr;
            
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (!cloud_queue_.empty()) {
                    msg = cloud_queue_.front();
                    cloud_queue_.pop();
                }
            }

            if (msg && !processing_.load()) {
                processing_.store(true);
                
                auto start_time = std::chrono::high_resolution_clock::now();
                
                // Process the cloud
                auto results = processCloud(msg);
                
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                
                // Store results thread-safely
                {
                    std::lock_guard<std::mutex> lock(results_mutex_);
                    latest_filtered_ = results.filtered;
                    latest_obstacles_ = results.obstacles;
                    latest_ground_ = results.ground;
                    has_results_ = true;
                }
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "Processing time: %ld ms", duration.count());
                
                processing_.store(false);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    struct ProcessResults {
        sensor_msgs::msg::PointCloud2 filtered;
        sensor_msgs::msg::PointCloud2 obstacles;
        sensor_msgs::msg::PointCloud2 ground;
    };

    ProcessResults processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        ProcessResults results;
        
        try {
            // Convert to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);

            if (cloud->empty()) return results;

            // OPTIMIZED Processing Pipeline
            auto filter_results = applyOptimizedFilters(cloud);

            // Convert back to ROS messages
            pcl::toROSMsg(*filter_results.filtered, results.filtered);
            pcl::toROSMsg(*filter_results.obstacles, results.obstacles);
            pcl::toROSMsg(*filter_results.ground, results.ground);

            // Set headers
            results.filtered.header = msg->header;
            results.obstacles.header = msg->header;
            results.ground.header = msg->header;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Processing error: %s", e.what());
        }

        return results;
    }

    // Publish at fixed rate for smooth visualization
    void publishResults()
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        
        if (has_results_) {
            filtered_3d_pub_->publish(latest_filtered_);
            obstacles_pub_->publish(latest_obstacles_);
            ground_pub_->publish(latest_ground_);
        }
    }

    // OPTIMIZED filtering pipeline
    struct FilterResults {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
    };

    FilterResults applyOptimizedFilters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        FilterResults results;
        results.filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        results.obstacles = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        results.ground = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        // 1. AGGRESSIVE Downsampling first (major performance boost)
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);  // 10cm resolution - faster processing
        vg.filter(*downsampled);

        // 2. Working area filter (remove distant points early)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(downsampled);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.0, 3.0);  // Height limits
        pass.filter(*results.filtered);

        pass.setInputCloud(results.filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-10.0, 10.0);  // Reduce working area for speed
        pass.filter(*results.filtered);

        // 3. Simple ground plane detection (optimized)
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(500);  // Reduced iterations for speed
        seg.setDistanceThreshold(0.15);  // Slightly more tolerant

        seg.setInputCloud(results.filtered);
        seg.segment(*inliers, *coefficients);

        if (!inliers->indices.empty()) {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(results.filtered);
            extract.setIndices(inliers);
            
            // Ground points
            extract.setNegative(false);
            extract.filter(*results.ground);
            
            // Obstacle points
            extract.setNegative(true);
            extract.filter(*results.obstacles);
        } else {
            *results.obstacles = *results.filtered;
        }

        return results;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AsyncPCLProcessor>());
    rclcpp::shutdown();
    return 0;
}
