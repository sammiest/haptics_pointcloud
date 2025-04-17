#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <math.h>
#include<vector>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <nanoflann.hpp>

using namespace Eigen;
using namespace nanoflann;


using namespace Eigen;

// These functions should all return a force in Newtons.
// Units:
//   position: mm
//   velocity: mm/s
//   force: N

Vector3d spring(const Vector3d& position, const Vector3d& velocity) {
    double k = 0.5; // Stiffness (N/mm)
    Vector3d force = Vector3d::Zero();
    // ============ Start of Your Code ============
    force = -k * position;
    // ============= End of Your Code =============
    return force;
}

Vector3d sphere(const Vector3d& position, const Vector3d& velocity) {
    double r = 50; // Sphere radius (mm)
    double k = 0.5; // Stiffness (N/mm)
    Vector3d force = Vector3d::Zero();
    // ============ Start of Your Code ============
    if (position.norm() <= r){
        double distance = r - position.norm();
        std::cout << distance << std::endl;
        force = k * distance * (position / position.norm());
    }

    // ============= End of Your Code =============
    return force;
}

Vector3d cube(const Vector3d& position, const Vector3d& velocity) {
    double k = 0.7; // Stiffness (N/mm)
    double cube_size = 50; // Length of edge of cube (mm)
    auto half_cube_size = cube_size / 2;
    Vector3d force = Vector3d::Zero();
    // ============ Start of Your Code ============
    if (position[0] > half_cube_size){
        force[0] = - k * (position[0]-half_cube_size);
    } else if (position[0] < -half_cube_size){
        force[0] = - k * (position[0]+half_cube_size);
    }

    if (position[1] > half_cube_size){
        force[1] = - k * (position[1]-half_cube_size);
    }
    else if (position[1] < -half_cube_size){
        force[1] = - k * (position[1]+half_cube_size);
    }

    if (position[2] > half_cube_size){
        force[2] = - k * (position[2]-half_cube_size);
    } else if (position[2] < -half_cube_size){
        force[2] = - k * (position[2]+half_cube_size);
    }
    // ============= End of Your Code =============
    return force;
}

Vector3d viscosity(const Vector3d& position, const Vector3d& velocity) {
    double b = 0.0025; // Viscosity (N/(mm/s))
    // VISCOSITY DO NOT EXCEED 0.0025!!
    double cube_size = 50; // Length of edge of cube (mm)
    auto half_cube_size = cube_size / 2;
    Vector3d force = Vector3d::Zero();
    // ============ Start of Your Code ============

    if(std::abs(position[0])< half_cube_size && std::abs(position[1]) < half_cube_size && std::abs(position[2]) < half_cube_size)
    {
        force = -b * velocity;
        std::cout << force << std::endl;
    }

    // ============= End of Your Code =============
    return force;
}

Vector3d friction(const Vector3d& position, const Vector3d& velocity) {
    double k = 0.6; // Stiffness (N/mm)
    double mu_large = 0.4; // Friction Coeff (NO UNIT)
    double mu_small = 0.2; // Friction Coeff (NO UNIT)
    double plane_size = 90; // Length of edge of square plane (mm)
    auto half_plane_size = plane_size / 2;
    auto friction_width = plane_size / 3; // Width of each friction zone (mm)
    double threshold = 5; // Max supported normal force before penetration (N)
    Vector3d force = Vector3d::Zero();
    // ============ Start of Your Code ============
    if (position[1] <= 0){


        if (position[0] < half_plane_size && position[0] > -half_plane_size && position[2] < half_plane_size && position[2] > -half_plane_size){
            force[1] = -k * (position[1]);

            if (position[0] > -half_plane_size && position[0] < -half_plane_size + friction_width){
                force[0] = 0;
            }
            else if (position[0] > -half_plane_size+friction_width && position[0] < -half_plane_size + 2*friction_width){
                force[0] = mu_small*force[1];
                force[2] = mu_small*force[1];

            }
            else if (position[0] > -half_plane_size+2*friction_width && position[0] < -half_plane_size + 3*friction_width){
                force[0] = mu_large*force[1];
                force[2] = mu_large*force[1];
            }
        }
    }

    if (force[1] > threshold){
        force[1] = 0;
    }
    

    // ============= End of Your Code =============
    return force;
}

Vector3d line(const Vector3d& position, const Vector3d& velocity) {
    double k = 0.5; // Stiffness (N/mm)
    Vector3d line_direction(1.0, 0.5, 0.2); // Line direction vector (x,y,z) (mm)
    Vector3d force = Vector3d::Zero();
    // ============ Start of Your Code ============
    // Hint: line passes through origin

    Vector3d dhat = line_direction/line_direction.norm();

    Vector3d pp = position.dot(dhat) * dhat;

    force = k * (pp - position);

    // ============= End of Your Code =============
    return force;
}

Eigen::Vector3d getPointOnCurve(double t, double mag) {
    double x = mag * cos(2 * t);
    double y = mag * sin(2 * t);
    double z = (mag / M_PI) * t;
    return Eigen::Vector3d(x, y, z);
}

Vector3d curve(const Vector3d& position, const Vector3d& velocity) {
    double k = 0.5; // Stiffness (N/mm)
    double mag = 30; // Circle radius observed along z-axis (mm)
    double cube_size = 120; // Length of edge of cube (mm)
    Vector3d force = Vector3d::Zero();
    // ============ Start of Your Code ============
    // Hint: only attracts to curve when stylus is within cube space
    // Hint: curve expression is in function getPointOnCurve()
    // NOTICE: t is in range [-PI, PI]
    double half_cube_size = cube_size/2;
    if(std::abs(position[0])< half_cube_size && std::abs(position[1]) < half_cube_size && std::abs(position[2]) < half_cube_size)
    {
        std::vector<Vector3d> curve_points;
        double increment = 2*M_PI / 100;
        double start = -M_PI;
        double current = start;
        for(int i = 0; i < 100; i++){
            current = start + i*increment;
            Vector3d curve_p = getPointOnCurve(current, mag);
            curve_points.push_back(curve_p);
        }

        double distance = 200;
        int index = 0;
        for (int j = 0; j < curve_points.size(); j++){
            double checkDistance = (position - curve_points[j]).norm();
            if(checkDistance < distance){
                distance = checkDistance;
                index = j;
            }
        }
        Vector3d d = curve_points[index] - position;
        if (d.norm() > 0.2){
            force = -k * (curve_points[index] - position);
        }

    }
    
    // ============= End of Your Code =============
    return force;
}

std::vector<Vector3d> pointCloud;

//for building the k-d tree
struct PointCloudAdaptor {
    std::vector<Vector3d>& points;
    PointCloudAdaptor(std::vector<Vector3d>& points) : points(points) {}

    inline size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return points[idx].x();
        else if (dim == 1) return points[idx].y();
        else return points[idx].z();
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};
bool flag = false;

#include <unordered_set>

// Custom hash for Vector3d using quantization to avoid float precision issues
struct Vector3dHash {
    size_t operator()(const Eigen::Vector3d& v) const {
        // Scale to integer grid to avoid floating point equality issues
        auto hash_val = [](double x) { return std::hash<int64_t>()(static_cast<int64_t>(x * 1e6)); };
        return hash_val(v.x()) ^ (hash_val(v.y()) << 1) ^ (hash_val(v.z()) << 2);
    }
};

struct Vector3dEqual {
    bool operator()(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
        return (a - b).norm() < 1e-6; // Near-equality
    }
};

void loadSTL(const std::string& filename) {
    std::ifstream stlFile(filename);
    if (!stlFile) {
        std::cerr << "Failed to open STL file\n";
        return;
    }

    double scaleX = 0.0005, scaleY = 0.0005, scaleZ = 0.0005;
    double angleX = 0.0, angleY = 0.0, angleZ = 0.0;
    double translateX = 0.0, translateY = 0.0, translateZ = 0.0;

    Eigen::Quaterniond q = Eigen::AngleAxisd(angleX, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(angleY, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(angleZ, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

    Eigen::Matrix3d scaledRotation;
    scaledRotation.col(0) = rotationMatrix.col(0) * scaleX;
    scaledRotation.col(1) = rotationMatrix.col(1) * scaleY;
    scaledRotation.col(2) = rotationMatrix.col(2) * scaleZ;

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = scaledRotation;
    transformation(0, 3) = translateX;
    transformation(1, 3) = translateY;
    transformation(2, 3) = translateZ;

    std::cout << "Transformation Matrix:\n" << transformation << "\n\n";

    std::string line;
    pointCloud.reserve(100000);

    std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual> uniquePoints;

    while (std::getline(stlFile, line)) {
        std::istringstream iss(line);
        std::string word;
        iss >> word;

        if (word == "v") {
            double x, y, z;
            iss >> x >> y >> z;

            Eigen::Vector4d point(x, y, z, 1.0);
            Eigen::Vector4d transformedPoint = transformation * point;
            Eigen::Vector3d result(transformedPoint.x(), transformedPoint.y(), transformedPoint.z());

            // Only add unique (or near-unique) points
            if (uniquePoints.insert(result).second) {
                pointCloud.emplace_back(result);
            }
        }
    }

    std::cout << "Loaded " << pointCloud.size() << " unique points.\n";
    flag = true;
}




Vector3d extra(const Vector3d& position, const Vector3d& velocity) {
    double k = 60.0;                      // Spring constant
    double damping = 0.8;                 // Damping factor
    double influence_radius = 15;       // Influence zone
    Vector3d force = Vector3d::Zero();

    if (!flag) {
        std::string stlFilePath = "C:\\Users\\FRBGuest\\Desktop\\Lab4\\medrob-lab4-main\\funnels\\UM3E_Funnel 7.obj";
        loadSTL(stlFilePath);

        if (pointCloud.empty()) {
            std::cerr << "STL point cloud is empty.\n";
            exit(0);
        }
    }

    // Build KD-Tree
    PointCloudAdaptor adaptor(pointCloud);
    typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloudAdaptor>, PointCloudAdaptor, 3> KDTree;
    KDTree index(3, adaptor, KDTreeSingleIndexAdaptorParams(10));
    index.buildIndex();

    const size_t num_results = 10;  // Use more neighbors for smoother normals
    double query_pt[3] = { position.x(), position.y(), position.z() };

    std::vector<unsigned int> ret_indexes(num_results); 
    std::vector<double> out_dists_sqr(num_results);

    index.knnSearch(&query_pt[0], num_results, ret_indexes.data(), out_dists_sqr.data());

    double min_dist = std::sqrt(out_dists_sqr[0]);
    std::cout << "Distance to surface: " << min_dist << std::endl;

    if (min_dist < influence_radius) {
        std::cout << "Influencing..." << std::endl;

        // Compute a stable averaged surface normal
        Vector3d normal_sum = Vector3d::Zero();
        Vector3d anchor = pointCloud[ret_indexes[0]];
        for (size_t i = 1; i + 1 < num_results; ++i) {
            Vector3d b = pointCloud[ret_indexes[i]];
            Vector3d c = pointCloud[ret_indexes[i + 1]];
            Vector3d n = (b - anchor).cross(c - anchor);
            if (n.norm() > 1e-6) {
                normal_sum += n.normalized();
            }
        }

        Vector3d normal = normal_sum.normalized();

        // Ensure the normal points outward
        Vector3d to_position = (position - anchor).normalized();
        if (normal.dot(to_position) < 0) {
            normal = -normal;
        }

        double penetration = influence_radius - min_dist;

        // Compute spring and damping force
        Vector3d spring_force = k * penetration * normal;
        Vector3d damping_force = -damping * velocity.dot(normal) * normal;

        force = spring_force + damping_force;

        std::cout << "Normal: " << normal.transpose() << "\n";
        std::cout << "Force: " << force.transpose() << "\n";
    }

    return force;
}
