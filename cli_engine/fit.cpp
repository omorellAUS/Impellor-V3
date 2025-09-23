// CLI Reverse-Engineering Engine (C++ with OCCT and PCL) - v3 (ImpellorV1 Integration)
// Merged OBJ loader from main.cpp; supports .stl, .obj, .ply; ICP, blade segmentation, etc.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

// OCCT includes
#include <Standard_Version.hxx>
#include <BRepBuilderAPI_MakeCylinder.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <STEPControl_Writer.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace fs = std::filesystem;
using json = nlohmann::json;

struct Vec3 {
    float x, y, z;
};

struct SimpleMesh {
    std::vector<Vec3> vertices;
    std::vector<std::vector<int>> faces;
    Vec3 minBound, maxBound;
};

SimpleMesh loadSimpleOBJ(const std::string& filename) {
    SimpleMesh mesh;
    std::ifstream file(filename);
    std::string line;

    mesh.minBound = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    mesh.maxBound = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            Vec3 v;
            iss >> v.x >> v.y >> v.z;
            mesh.vertices.push_back(v);
            mesh.minBound.x = std::min(mesh.minBound.x, v.x);
            mesh.minBound.y = std::min(mesh.minBound.y, v.y);
            mesh.minBound.z = std::min(mesh.minBound.z, v.z);
            mesh.maxBound.x = std::max(mesh.maxBound.x, v.x);
            mesh.maxBound.y = std::max(mesh.maxBound.y, v.y);
            mesh.maxBound.z = std::max(mesh.maxBound.z, v.z);
        } else if (type == "f") {
            std::vector<int> face;
            std::string token;
            while (iss >> token) {
                size_t pos = token.find('/');
                int idx = std::stoi(token.substr(0, pos));
                if (idx < 0) idx += mesh.vertices.size() + 1;
                face.push_back(idx - 1); // Convert to 0-based indexing
            }
            mesh.faces.push_back(face);
        }
    }

    std::cout << "Simple OBJ loaded: Vertices=" << mesh.vertices.size() << ", Faces=" << mesh.faces.size() << std::endl;
    return mesh;
}

struct BladeParams {
    double angle;
    double thickness;
    std::vector<gp_Pnt> camber_line;
};

void alignPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, double max_distance, int max_iterations);
void segmentBlades(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const gp_Dir& axis, const gp_Pln& backplate, const gp_Pln& shroud, int blade_count, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& blade_clouds, const std::string& output_dir);
void estimateAxisAndBore(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, gp_Dir& axis, double& radius, gp_Pnt& center);
void estimatePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const gp_Dir& axis, gp_Pln& backplate, gp_Pln& shroud);
void extractCurveNetwork(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const gp_Dir& axis, const gp_Pln& backplate, const gp_Pln& shroud, int section_count, std::vector<std::vector<gp_Pnt>>& sections);
TopoDS_Shape fitBladeSurfaces(const std::vector<std::vector<gp_Pnt>>& sections, double tolerance);
TopoDS_Shape createAnalyticDatums(const gp_Dir& axis, double radius, const gp_Pnt& center, const gp_Pln& backplate, const gp_Pln& shroud, const std::vector<TopoDS_Shape>& blade_surfaces, double fillet_radius);
void computeBladeParams(const std::vector<std::vector<gp_Pnt>>& sections, BladeParams& params);
void writeDeviationCSVandPLY(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const TopoDS_Shape& shape, const std::string& csv_path, const std::string& ply_path, double tolerance);
void writeParameterReport(const std::vector<BladeParams>& params, const std::string& json_path, const std::string& csv_path);

int main(int argc, char* argv[]) {
    std::cout << "Impellor V3 CLI Engine (Integrated with v1 OBJ Loader)" << std::endl;

    std::string config_path, output_step, output_json;
    for (int i = 1; i < argc; i += 2) {
        std::string arg = argv[i];
        if (arg == "--config") config_path = argv[i+1];
        else if (arg == "--output_step") output_step = argv[i+1];
        else if (arg == "--output_json") output_json = argv[i+1];
    }
    if (config_path.empty() || output_step.empty() || output_json.empty()) {
        std::cerr << "Usage: " << argv[0] << " --config <json> --output_step <step> --output_json <json>" << std::endl;
        return 1;
    }

    std::ifstream config_file(config_path);
    json config;
    config_file >> config;
    std::vector<std::string> mesh_paths = config["mesh_paths"].get<std::vector<std::string>>();
    double tolerance = config["tolerance"];
    int blade_count = config["blade_count"];
    double fillet_radius = config["fillet_radius"];
    int section_count = config["section_count"];
    double icp_max_distance = config.value("icp_max_distance", 0.1);
    int icp_max_iterations = config.value("icp_max_iterations", 100);

    // Load meshes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    for (const auto& mesh_path : mesh_paths) {
        std::string extension = mesh_path.substr(mesh_path.find_last_of(".") + 1);
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        pcl::PolygonMesh mesh;
        int load_result = -1;
        if (extension == "stl") {
            load_result = pcl::io::loadPolygonFileSTL(mesh_path, mesh);
        } else if (extension == "obj") {
            // Use PCL for OBJ, fallback to loadSimpleOBJ if needed
            load_result = pcl::io::loadPolygonFileOBJ(mesh_path, mesh);
            if (load_result != 0) {
                std::cerr << "PCL OBJ load failed, trying simple loader for: " << mesh_path << std::endl;
                SimpleMesh simple_mesh = loadSimpleOBJ(mesh_path);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                for (const auto& v : simple_mesh.vertices) {
                    cloud->push_back(pcl::PointXYZ(v.x, v.y, v.z));
                }
                clouds.push_back(cloud);
                continue;
            }
        } else if (extension == "ply") {
            load_result = pcl::io::loadPolygonFilePLY(mesh_path, mesh);
        } else {
            std::cerr << "Unsupported format: " << extension << std::endl;
            return 1;
        }
        if (load_result != 0) {
            std::cerr << "Failed to load mesh: " << mesh_path << std::endl;
            return 1;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
        clouds.push_back(cloud);
        std::cout << "Loaded cloud with " << cloud->size() << " points from " << mesh_path << std::endl;
    }

    // Align clouds using ICP
    alignPointClouds(clouds, icp_max_distance, icp_max_iterations);

    // Combine clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& cloud : clouds) {
        *combined_cloud += *cloud;
    }
    std::cout << "Combined cloud with " << combined_cloud->size() << " points." << std::endl;

    // Clean mesh
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(combined_cloud);
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.filter(*combined_cloud);

    pcl::MovingLeastSquares<pcl::PointXYZ> mls;
    mls.setInputCloud(combined_cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchRadius(0.5);
    mls.process(*combined_cloud);

    // Estimate bore
    gp_Dir axis(0, 0, 1);
    double radius = 10.0;
    gp_Pnt center(0, 0, 0);
    estimateAxisAndBore(combined_cloud, axis, radius, center);

    // Estimate planes
    gp_Pln backplate(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    gp_Pln shroud(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    estimatePlanes(combined_cloud, axis, backplate, shroud);

    // Segment blades
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> blade_clouds;
    std::string output_dir = output_step.substr(0, output_step.find_last_of("/\\"));
    segmentBlades(combined_cloud, axis, backplate, shroud, blade_count, blade_clouds, output_dir);
    if (blade_clouds.size() != static_cast<size_t>(blade_count)) {
        std::cerr << "Expected " << blade_count << " blades, but found " << blade_clouds.size() << std::endl;
        return 1;
    }

    // Process each blade
    std::vector<TopoDS_Shape> blade_surfaces;
    std::vector<BladeParams> blade_params;
    for (size_t i = 0; i < blade_clouds.size(); ++i) {
        std::vector<std::vector<gp_Pnt>> blade_sections;
        extractCurveNetwork(blade_clouds[i], axis, backplate, shroud, section_count, blade_sections);
        blade_surfaces.push_back(fitBladeSurfaces(blade_sections, tolerance));

        BladeParams params;
        computeBladeParams(blade_sections, params);
        blade_params.push_back(params);
    }

    // Create datums and apply fillets
    TopoDS_Shape datums = createAnalyticDatums(axis, radius, center, backplate, shroud, blade_surfaces, fillet_radius);

    // Sew with gap repair
    BRepBuilderAPI_Sewing sewer(tolerance);
    sewer.Add(datums);
    sewer.SetMinTolerance(tolerance / 2);
    sewer.SetMaxTolerance(tolerance * 2);
    sewer.Perform();
    TopoDS_Shape sewn_shape = sewer.SewedShape();

    // Deviation map
    std::string csv_path = output_dir + "/deviation.csv";
    std::string ply_path = output_dir + "/deviation.ply";
    writeDeviationCSVandPLY(combined_cloud, sewn_shape, csv_path, ply_path, tolerance);

    // Parameter report
    std::string param_json_path = output_dir + "/parameters.json";
    std::string param_csv_path = output_dir + "/parameters.csv";
    writeParameterReport(blade_params, param_json_path, param_csv_path);

    // Export STEP
    STEPControl_Writer writer;
    writer.Transfer(sewn_shape, STEPControl_AsIs);
    if (writer.Write(output_step.c_str()) != IFSelect_RetDone) {
        std::cerr << "Failed to write STEP: " << output_step << std::endl;
        return 1;
    }

    // Output JSON
    json output_data = {
        {"step_path", output_step},
        {"axis_vector", {{"x", axis.X()}, {"y", axis.Y()}, {"z", axis.Z()}}},
        {"bore_radius", radius},
        {"bore_center", {{"x", center.X()}, {"y", center.Y()}, {"z", center.Z()}}},
        {"backplate_normal", {{"x", backplate.Axis().Direction().X()}, {"y", backplate.Axis().Direction().Y()}, {"z", backplate.Axis().Direction().Z()}}},
        {"backplate_position", {{"x", backplate.Location().X()}, {"y", backplate.Location().Y()}, {"z", backplate.Location().Z()}}},
        {"shroud_normal", {{"x", shroud.Axis().Direction().X()}, {"y", shroud.Axis().Direction().Y()}, {"z", shroud.Axis().Direction().Z()}}},
        {"shroud_position", {{"x", shroud.Location().X()}, {"y", shroud.Location().Y()}, {"z", shroud.Location().Z()}}},
        {"blade_count", blade_count}
    };
    std::ofstream output_file(output_json);
    output_file << output_data.dump(4);

    return 0;
}

void alignPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, double max_distance, int max_iterations) {
    if (clouds.size() <= 1) {
        std::cout << "Only one cloud; skipping ICP alignment." << std::endl;
        return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(max_iterations);
    icp.setMaxCorrespondenceDistance(max_distance);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.01);

    for (size_t i = 1; i < clouds.size(); ++i) {
        icp.setInputSource(clouds[i]);
        icp.setInputTarget(clouds[0]);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);
        if (icp.hasConverged()) {
            std::cout << "ICP converged for cloud " << i << ", score: " << icp.getFitnessScore() << std::endl;
            *clouds[i] = aligned;
        } else {
            std::cerr << "ICP failed for cloud " << i << std::endl;
        }
    }
}

void segmentBlades(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const gp_Dir& axis, const gp_Pln& backplate, const gp_Pln& shroud, int blade_count, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& blade_clouds, const std::string& output_dir) {
    // Remove backplate and shroud points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    double back_z = backplate.Location().Z();
    double shroud_z = shroud.Location().Z();
    double margin = 0.5;  // 0.5 mm buffer
    for (const auto& pt : *cloud) {
        if (pt.z > back_z + margin && pt.z < shroud_z - margin) {
            filtered_cloud->push_back(pt);
        }
    }

    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud(filtered_cloud);
    norm_est.setRadiusSearch(0.5);
    norm_est.compute(*normals);

    // Euclidean clustering (coarse)
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(1.0);  // 1 mm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(cloud->size() / 2);
    ec.setInputCloud(filtered_cloud);
    ec.extract(clusters);

    // Refine with region growing
    blade_clouds.clear();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(cloud->size() / 2);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);  // 5 degrees
    reg.setCurvatureThreshold(1.0);

    for (size_t i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::Normal>::Ptr cluster_normals(new pcl::PointCloud<pcl::Normal>());
        for (const auto& idx : clusters[i].indices) {
            cluster_cloud->push_back((*filtered_cloud)[idx]);
            cluster_normals->push_back((*normals)[idx]);
        }

        reg.setInputCloud(cluster_cloud);
        reg.setInputNormals(cluster_normals);
        std::vector<pcl::PointIndices> refined_clusters;
        reg.extract(refined_clusters);

        for (size_t j = 0; j < refined_clusters.size(); ++j) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr blade_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto& idx : refined_clusters[j].indices) {
                blade_cloud->push_back((*cluster_cloud)[idx]);
            }
            blade_clouds.push_back(blade_cloud);
            // Save blade cloud for debugging
            std::string blade_ply = output_dir + "/blade_" + std::to_string(blade_clouds.size() - 1) + ".ply";
            pcl::io::savePLYFile(blade_ply, *blade_cloud);
            std::cout << "Saved blade cloud " << blade_clouds.size() - 1 << " to " << blade_ply << std::endl;
        }
    }

    // Validate blade count
    if (blade_clouds.size() > static_cast<size_t>(blade_count)) {
        std::sort(blade_clouds.begin(), blade_clouds.end(), 
            [](const auto& a, const auto& b) { return a->size() > b->size(); });
        blade_clouds.resize(blade_count);
    }
}

void estimateAxisAndBore(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, gp_Dir& axis, double& radius, gp_Pnt& center) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(5.0, 50.0);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "No cylinder found; using PCA." << std::endl;
        Eigen::Vector4f pca_centroid;
        pcl::compute3DCentroid(*cloud, pca_centroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pca_centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();
        Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
        axis = gp_Dir(eigen_vectors.col(0)[0], eigen_vectors.col(0)[1], eigen_vectors.col(0)[2]);
        center = gp_Pnt(pca_centroid[0], pca_centroid[1], pca_centroid[2]);
        radius = 10.0;
        return;
    }

    center = gp_Pnt(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    axis = gp_Dir(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    radius = coefficients->values[6];
}

void estimatePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const gp_Dir& axis, gp_Pln& backplate, gp_Pln& shroud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(cloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setMaxIterations(1000);
    Eigen::Vector3f axis_eigen(axis.X(), axis.Y(), axis.Z());
    seg.setAxis(axis_eigen);
    seg.setEpsAngle(0.1);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
        std::cerr << "No plane found." << std::endl;
        return;
    }

    gp_Dir normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    double d = coefficients->values[3];
    gp_Pnt point_on_plane(0, 0, -d / coefficients->values[2]);
    backplate = gp_Pln(point_on_plane, normal);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
        std::cerr << "No second plane found." << std::endl;
        return;
    }

    normal = gp_Dir(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    d = coefficients->values[3];
    point_on_plane = gp_Pnt(0, 0, -d / coefficients->values[2]);
    shroud = gp_Pln(point_on_plane, normal);

    double back_pos = backplate.Location().Z();
    double shroud_pos = shroud.Location().Z();
    if (back_pos > shroud_pos) {
        std::swap(backplate, shroud);
    }
}

void extractCurveNetwork(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const gp_Dir& axis, const gp_Pln& backplate, const gp_Pln& shroud, int section_count, std::vector<std::vector<gp_Pnt>>& sections) {
    double back_z = backplate.Location().Z();
    double shroud_z = shroud.Location().Z();
    double step = (shroud_z - back_z) / (section_count - 1);

    sections.resize(section_count);
    for (int i = 0; i < section_count; ++i) {
        double z = back_z + i * step;
        gp_Pln slice_plane(gp_Pnt(0, 0, z), axis);

        pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients());
        plane_coeffs->values = {float(axis.X()), float(axis.Y()), float(axis.Z()), float(-z)};
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(plane_coeffs);
        proj.filter(*projected);

        sections[i].reserve(projected->size());
        for (const auto& pt : *projected) {
            sections[i].push_back(gp_Pnt(pt.x, pt.y, pt.z));
        }
    }
}

TopoDS_Shape fitBladeSurfaces(const std::vector<std::vector<gp_Pnt>>& sections, double tolerance) {
    int u_degree = 3, v_degree = 3;
    int u_points = sections[0].size();
    int v_points = sections.size();
    TColgp_Array2OfPnt points(1, u_points, 1, v_points);
    for (int v = 0; v < v_points; ++v) {
        for (int u = 0; u < u_points; ++u) {
            points.SetValue(u + 1, v + 1, sections[v][u]);
        }
    }

    GeomAPI_PointsToBSplineSurface surface_fitter(points, u_degree, v_degree, GeomAbs_C2, tolerance);
    Handle(Geom_BSplineSurface) surface = surface_fitter.Surface();
    return BRepBuilderAPI_MakeFace(surface, tolerance).Face();
}

TopoDS_Shape createAnalyticDatums(const gp_Dir& axis, double radius, const gp_Pnt& center, const gp_Pln& backplate, const gp_Pln& shroud, const std::vector<TopoDS_Shape>& blade_surfaces, double fillet_radius) {
    gp_Ax2 ax(center, axis);
    TopoDS_Shape cylinder = BRepBuilderAPI_MakeCylinder(ax, radius, 100.0).Shape();
    TopoDS_Face back_face = BRepBuilderAPI_MakeFace(backplate);
    TopoDS_Face shroud_face = BRepBuilderAPI_MakeFace(shroud);

    std::vector<TopoDS_Shape> filleted_blades;
    for (const auto& blade : blade_surfaces) {
        BRepFilletAPI_MakeFillet fillet_maker(blade);
        for (TopoDS_Iterator it(blade); it.More(); it.Next()) {
            TopoDS_Edge edge = TopoDS::Edge(it.Value());
            fillet_maker.Add(fillet_radius, edge);
        }
        filleted_blades.push_back(fillet_maker.Shape());
    }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    builder.Add(compound, cylinder);
    builder.Add(compound, back_face);
    builder.Add(compound, shroud_face);
    for (const auto& blade : filleted_blades) {
        builder.Add(compound, blade);
    }

    return compound;
}

void computeBladeParams(const std::vector<std::vector<gp_Pnt>>& sections, BladeParams& params) {
    gp_Pnt leading = sections[0][0];
    gp_Pnt trailing = sections[0].back();
    gp_Vec chord(leading, trailing);
    params.angle = atan2(chord.Y(), chord.X()) * 180.0 / M_PI;

    double max_thickness = 0.0;
    for (const auto& section : sections) {
        for (size_t i = 1; i < section.size() - 1; ++i) {
            double dist = section[i].Distance(section[i - 1]);
            max_thickness = std::max(max_thickness, dist);
        }
    }
    params.thickness = max_thickness;

    params.camber_line.clear();
    for (const auto& section : sections) {
        gp_Pnt mid = section[section.size() / 2];
        params.camber_line.push_back(mid);
    }
}

void writeDeviationCSVandPLY(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const TopoDS_Shape& shape, const std::string& csv_path, const std::string& ply_path, double tolerance) {
    std::ofstream csv(csv_path);
    csv << "point_x,point_y,point_z,deviation\n";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& point : *cloud) {
        BRepExtrema_DistShapeShape dist(gp_Pnt(point.x, point.y, point.z), shape);
        double dev = dist.Value();
        csv << point.x << "," << point.y << "," << point.z << "," << dev << "\n";

        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        if (dev < tolerance / 2) {
            colored_point.r = 0; colored_point.g = 255; colored_point.b = 0;
        } else if (dev < tolerance) {
            colored_point.r = 255; colored_point.g = 255; colored_point.b = 0;
        } else {
            colored_point.r = 255; colored_point.g = 0; colored_point.b = 0;
        }
        colored_cloud->push_back(colored_point);
    }
    pcl::io::savePLYFile(ply_path, *colored_cloud);
}

void writeParameterReport(const std::vector<BladeParams>& params, const std::string& json_path, const std::string& csv_path) {
    json param_data = json::array();
    for (size_t i = 0; i < params.size(); ++i) {
        json blade = {
            {"blade_index", i},
            {"blade_angle_deg", params[i].angle},
            {"max_thickness_mm", params[i].thickness},
            {"camber_line", json::array()}
        };
        for (const auto& pt : params[i].camber_line) {
            blade["camber_line"].push_back({{"x", pt.X()}, {"y", pt.Y()}, {"z", pt.Z()}});
        }
        param_data.push_back(blade);
    }
    std::ofstream json_file(json_path);
    json_file << param_data.dump(4);

    std::ofstream csv(csv_path);
    csv << "blade_index,blade_angle_deg,max_thickness_mm,camber_line_points\n";
    for (size_t i = 0; i < params.size(); ++i) {
        csv << i << "," << params[i].angle << "," << params[i].thickness << ",";
        for (size_t j = 0; j < params[i].camber_line.size(); ++j) {
            const auto& pt = params[i].camber_line[j];
            csv << pt.X() << ":" << pt.Y() << ":" << pt.Z();
            if (j < params[i].camber_line.size() - 1) csv << ";";
        }
        csv << "\n";
    }
}
