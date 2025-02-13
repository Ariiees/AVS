#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"

#include <chrono>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

class PclFilterNode : public rclcpp::Node
{
public:
    PclFilterNode() : Node("pcl_filter_node")
    {
        // Subscriber to point cloud data
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/top/pointcloud_raw", 10, std::bind(&PclFilterNode::pointCloudCallback, this, std::placeholders::_1)
        );

        // Publisher for the filtered point cloud
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("AVS/filtered_point_cloud", 10);

        // Timer to report benchmark every second
        // benchmark_timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1),
        //     std::bind(&PclFilterNode::benchmarkMetrics, this)
        // );
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Start timing the processing
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // TODO: convert spend lots of time, how to measure the conversion time?
        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Apply VoxelGrid filter (Downsampling)
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(pcl_cloud);
        // FIX: [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
        voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2); 
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid_filter.filter(*voxel_filtered_cloud);

        // // Apply StatisticalOutlierRemoval filter (Remove outliers)
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
        // sor_filter.setInputCloud(voxel_filtered_cloud);
        // sor_filter.setMeanK(50); // Number of nearest neighbors to analyze for each point
        // sor_filter.setStddevMulThresh(1.0); // Standard deviation threshold
        // pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // sor_filter.filter(*sor_filtered_cloud);

        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*voxel_filtered_cloud, output_msg);

        // Publish the filtered point cloud
        point_cloud_pub_->publish(output_msg);

        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> processing_duration = end_time - start_time;
        processing_time_ = processing_duration.count();  // Store processing time
        RCLCPP_INFO(this->get_logger(), "%.4f,%lu", 
            processing_time_, 
            voxel_filtered_cloud->size());
        // Original Points: 230400 points
        // RCLCPP_INFO(this->get_logger(), "Processing Time: %.4f seconds", processing_time_);
        // RCLCPP_INFO(this->get_logger(), "Original Point Cloud Size: %lu points", pcl_cloud->size());
        // RCLCPP_INFO(this->get_logger(), "After VoxelGrid Filter: %lu points", voxel_filtered_cloud->size());
        // RCLCPP_INFO(this->get_logger(), "After StatisticalOutlierRemoval Filter: %lu points", sor_filtered_cloud->size());

    }

    // Function to benchmark CPU and memory usage
    void benchmarkMetrics()
    {
        // Get CPU usage
        float cpu_usage = getCpuUsage();

        // Get Memory usage (in MB)
        float memory_usage = getMemoryUsage();

        // Print benchmark information
        RCLCPP_INFO(this->get_logger(), "Processing Time: %.4f seconds", processing_time_);
        RCLCPP_INFO(this->get_logger(), "CPU Usage: %.2f%%", cpu_usage);
        RCLCPP_INFO(this->get_logger(), "Memory Usage: %.2f MB", memory_usage);
    }

    // Function to get CPU usage
    float getCpuUsage()
    {
        static long prev_idle = 0, prev_total = 0;
        std::ifstream proc_stat("/proc/stat");
        std::string line;
        std::getline(proc_stat, line);
        proc_stat.close();

        std::istringstream ss(line);
        std::string cpu;
        long user, nice, system, idle, iowait, irq, softirq, steal;
        ss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

        long idle_time = idle + iowait;
        long total_time = user + nice + system + idle_time + irq + softirq + steal;

        long delta_idle = idle_time - prev_idle;
        long delta_total = total_time - prev_total;

        prev_idle = idle_time;
        prev_total = total_time;

        return delta_total > 0 ? (100.0f * (delta_total - delta_idle) / delta_total) : 0.0f;
    }

    // Function to get memory usage
    float getMemoryUsage()
    {
        std::ifstream proc_status("/proc/self/status");
        std::string line;
        while (std::getline(proc_status, line)) {
            if (line.find("VmRSS:") == 0) {
                std::istringstream ss(line);
                std::string label;
                int memory_kb;
                ss >> label >> memory_kb;
                return memory_kb / 1024.0f;  // Convert KB to MB
            }
        }
        return 0.0f;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::TimerBase::SharedPtr benchmark_timer_;

    double processing_time_;  // To store processing time for benchmarking
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclFilterNode>());
    rclcpp::shutdown();
    return 0;
}
