```cpp
#include <iostream>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/region_growing.h>
#include <TopoDS_Shape.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        return 1;
    }

    // Validate config file
    std::string config_path = argv[1];
    if (!std::filesystem::exists(config_path)) {
        std::cerr << "Error: Config file not found: " << config_path << std::endl;
        return 1;
    }

    // Read configuration
    std::ifstream config_file(config_path);
    json config;
    try {
        config_file >> config;
    } catch (const json::exception& e) {
        std::cerr << "Error parsing config file: " << e.what() << std::endl;
        return 1;
    }

    // Set default values
    std::string input_path = config.value("input_path", "test.obj");
    std::string output_path = config.value("output_path", "output/fit.pcd");
    int blade_count = config.value("blade_count", 8);
    int icp_max_iterations = config.value("icp_max_iterations", 100);
    double icp_max_distance = config.value("icp_max_correspondence_distance", 0.01);
    double ransac_threshold = config.value("ransac_distance_threshold", 0.02);

    // Validate input file
    if (!std::filesystem::exists(input_path)) {
        std::cerr << "Error: Input file not found: " << input_path << std::endl;
        return 1;
    }

    // Create output directory
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());

    // Load OBJ file
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    try {
        if (pcl::io::loadOBJFile(input_path, *cloud) < 0) {
            std::cerr << "Error: Failed to load OBJ file: " << input_path << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading OBJ file: " << e.what() << std::endl;
        return 1;
    }

    // Basic mesh validation
    if (cloud->empty()) {
        std::cerr << "Error: Loaded mesh is empty" << std::endl;
        return 1;
    }

    // Compute normals if missing
    if (!cloud->hasNormals()) {
        std::cerr << "Warning: OBJ file has no normals, computing normals..." << std::endl;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud->makeShared());
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.05);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*normals);
        pcl::concatenateFields(*cloud, *normals, *cloud);
    }

    // Smooth point cloud using MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointNormal>);
    try {
        pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
        mls.setInputCloud(cloud);
        mls.setPolynomialFit(true);
        mls.setSearchRadius(0.05);
        mls.process(*smoothed_cloud);
    } catch (const std::exception& e) {
        std::cerr << "Error during MLS smoothing: " << e.what() << std::endl;
        return 1;
    }

    // Save output
    try {
        pcl::io::savePCDFileASCII(output_path, *smoothed_cloud);
        std::cout << "Processed mesh saved to: " << output_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error saving PCD file: " << e.what() << std::endl;
        return 1;
    }

    // Placeholder for further processing
    std::cout << "Processing complete. Blade count: " << blade_count 
              << ", ICP iterations: " << icp_max_iterations 
              << ", ICP distance: " << icp_max_distance 
              << ", RANSAC threshold: " << ransac_threshold << std::endl;

    return 0;
}
```
