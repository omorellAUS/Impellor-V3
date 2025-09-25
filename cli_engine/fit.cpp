--- cli_engine/fit.cpp
+++ cli_engine/fit.cpp
@@ -1,5 +1,6 @@
 #include <pcl/io/pcd_io.h>
 #include <pcl/io/stl_io.h>
+#include <pcl/segmentation/euclidean_cluster_extraction.h>
 #include <nlohmann/json.hpp>
 #include <iostream>
 #include <string>
@@ -10,6 +11,8 @@
 int main(int argc, char** argv) {
     if (argc != 2) {
         std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
         return -1;
     }

     // Load config.json
     std::ifstream config_file(argv[1]);
     nlohmann::json config;
     config_file >> config;

     // Load mesh
     pcl::PolygonMesh mesh;
     pcl::io::loadPolygonFileSTL(config["input_path"].get<std::string>(), mesh);

     // Perform blade segmentation (e.g., Euclidean clustering)
-    int expected_blade_count = config["blade_count"].get<int>();
+    int blade_count_min = config.value("blade_count_min", 8);
+    int blade_count_max = config.value("blade_count_max", 20);
+
+    // Example: Euclidean clustering for blade segmentation
+    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
+    // Convert mesh to point cloud (simplified, adjust as per actual code)
+    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
+
+    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
+    std::vector<pcl::PointIndices> cluster_indices;
+    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
+    ec.setClusterTolerance(0.02); // Adjust based on mesh scale
+    ec.setMinClusterSize(100);    // Minimum points per blade
+    ec.setMaxClusterSize(10000);  // Maximum points per blade
+    ec.setSearchMethod(tree);
+    ec.setInputCloud(cloud);
+    ec.extract(cluster_indices);
+
+    int detected_blade_count = cluster_indices.size();
+    if (detected_blade_count < blade_count_min || detected_blade_count > blade_count_max) {
+        std::cerr << "Error: Detected " << detected_blade_count << " blades, expected between "
+                  << blade_count_min << " and " << blade_count_max << std::endl;
+        return -1;
+    }
+    std::cout << "Detected " << detected_blade_count << " blades" << std::endl;

     // Continue with NURBS fitting, STEP export, etc.
     // ...
     return 0;
 }
