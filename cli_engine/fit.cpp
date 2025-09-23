```cpp
#include <iostream>
#include <filesystem>
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

    // Read configuration
    std::ifstream config_file(argv[1]);
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open config file: " << argv[1] << std::endl;
        return 1;
    }
    json config;
    try {
        config_file >> config;
    } catch (const json::exception& e) {
        std::cerr << "Error parsing config file: " << e.what() << std::endl;
        return 1;
    }

    std::string input_path = config["input_path"];
    std::string output_path = config["output_path"];
    int blade_count = config["blade_count"];
    int icp_max_iterations = config["icp_max_iterations"];
    double icp_max_distance = config["icp_max_correspondence_distance"];
    double ransac_threshold = config["ransac_distance_threshold"];

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

    // Example processing: Smooth point cloud using MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchRadius(0.05);
    mls.process(*smoothed_cloud);

    // Save output
    try {
        pcl::io::savePCDFileASCII(output_path, *smoothed_cloud);
        std::cout << "Processed mesh saved to: " << output_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error saving PCD file: " << e.what() << std::endl;
        return 1;
    }

    // Placeholder for further processing (ICP, segmentation, NURBS, STEP export)
    std::cout << "Processing complete. Blade count: " << blade_count 
              << ", ICP iterations: " << icp_max_iterations 
              << ", ICP distance: " << icp_max_distance 
              << ", RANSAC threshold: " << ransac_threshold << std::endl;

    return 0;
}
```
