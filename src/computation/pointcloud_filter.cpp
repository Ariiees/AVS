#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace fs = std::filesystem;

struct PointXYZI {
    float x, y, z, intensity;
};

// Function to read a .bin file into a PCL point cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr readBinFile(const std::string& filePath) {
    std::cout << "Reading file: " << filePath << std::endl;
    std::ifstream inputFile(filePath, std::ios::binary);
    if (!inputFile.is_open()) {
        throw std::runtime_error("Could not open file: " + filePath);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    size_t pointCount = 0;

    while (true) {
        PointXYZI point;
        inputFile.read(reinterpret_cast<char*>(&point.x), sizeof(float));
        
        if (inputFile.gcount() != sizeof(float)) {
            break;  // If not exactly 4 bytes, it means we've reached the end of the data
        }

        inputFile.read(reinterpret_cast<char*>(&point.y), sizeof(float));
        if (inputFile.gcount() != sizeof(float)) break;

        inputFile.read(reinterpret_cast<char*>(&point.z), sizeof(float));
        if (inputFile.gcount() != sizeof(float)) break;

        inputFile.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
        if (inputFile.gcount() != sizeof(float)) break;

        pcl::PointXYZI pclPoint;
        pclPoint.x = point.x;
        pclPoint.y = point.y;
        pclPoint.z = point.z;
        pclPoint.intensity = point.intensity;
        cloud->push_back(pclPoint);
        pointCount++;
    }

    inputFile.close();
    std::cout << "File read successfully. Total points: " << pointCount << std::endl;
    return cloud;
}


// Function to write a PCL point cloud to a .bin file
void writeBinFile(const std::string& filePath, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    std::cout << "Writing file: " << filePath << " with " << cloud->size() << " points." << std::endl;
    std::ofstream outputFile(filePath, std::ios::binary);
    if (!outputFile.is_open()) {
        throw std::runtime_error("Could not open file: " + filePath);
    }

    for (const auto& point : cloud->points) {
        outputFile.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
        outputFile.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
        outputFile.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
        outputFile.write(reinterpret_cast<const char*>(&point.intensity), sizeof(float));
    }

    outputFile.close();
    std::cout << "File written successfully." << std::endl;
}

// Function to process a point cloud using VoxelGrid and StatisticalOutlierRemoval filters
pcl::PointCloud<pcl::PointXYZI>::Ptr processPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    // std::cout << "Processing point cloud with " << cloud->size() << " points." << std::endl;

    // Downsample the point cloud using VoxelGrid filters
    // pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
    // voxelGrid.setInputCloud(cloud);
    // voxelGrid.setLeafSize(1.0f, 1.0f, 1.0f); // Adjust the leaf size
    // pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZI>());
    // voxelGrid.filter(*downsampledCloud);
    // std::cout << "Downsampling complete. Points reduced to: " << downsampledCloud->size() << std::endl;

    // Remove outliers using StatisticalOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50); // Number of nearest neighbors to use for mean distance estimation
    sor.setStddevMulThresh(1.0); // Set threshold multiplier
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>());
    sor.filter(*filteredCloud);
    std::cout << "Outlier removal complete. Points reduced to: " << filteredCloud->size() << std::endl;

    return filteredCloud;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_folder> <output_folder>\n";
        return EXIT_FAILURE;
    }

    std::string inputFolder = argv[1];
    std::string outputFolder = argv[2];

    fs::create_directories(outputFolder);
    std::cout << "Output folder created: " << outputFolder << std::endl;

    for (const auto& entry : fs::directory_iterator(inputFolder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".bin") {
            std::string inputFilePath = entry.path().string();
            std::string outputFilePath = outputFolder + "/" + entry.path().filename().string();
            try {
                std::cout << "Processing file: " << inputFilePath << std::endl;

                auto cloud = readBinFile(inputFilePath);
                if (cloud->empty()) {
                    std::cerr << "Warning: Empty point cloud from file: " << inputFilePath << std::endl;
                    continue;
                }
                auto processedCloud = processPointCloud(cloud);
                writeBinFile(outputFilePath, processedCloud);

                std::cout << "Processed successfully: " << inputFilePath << " -> " << outputFilePath << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Error processing file " << inputFilePath << ": " << e.what() << "\n";
            }
        } else {
            std::cout << "Skipping non-bin file: " << entry.path().string() << std::endl;
        }
    }

    std::cout << "Processing completed for all files in: " << inputFolder << std::endl;
    return EXIT_SUCCESS;
}
