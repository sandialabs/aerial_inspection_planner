#pragma once
#include <cstdlib>
#include <fstream>
#include <cfloat>
#include <Eigen/Dense>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "rclcpp/rclcpp.hpp"

/**
*  \brief This function reads an ACII STL file for inspection planning

    From https://github.com/ethz-asl/StructuralInspectionPlanner
*/
namespace mesh_utils
{

struct Triangle {
    std::array<Eigen::Vector3f, 3> vertices;
};

bool ray_intersects_triangle(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_direction, const Triangle& triangle);
bool is_point_inside_mesh(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDirection, const std::vector<Triangle>& mesh);
bool segment_intersects_triangle(const Eigen::Vector3f& P0, const Eigen::Vector3f& P1, const Triangle& triangle);

std::unique_ptr<std::vector<Triangle>> readSTLfile(std::string name, rclcpp::Logger logger_);

Eigen::Vector3f calculate_surface_normal(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

Eigen::Vector3f calculate_centroid(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

Eigen::Quaternionf vector_to_quaternion(const Eigen::Vector3f& unit_vector);

double vector_to_yaw(const Eigen::Vector3f& unit_vector);

// Function to compute the angle between two vectors in degrees
float angle_between_vectors(const Eigen::Vector3f& u, const Eigen::Vector3f& v);

/*
Checks
- Point within vp fov
- Point within incendent angle from surface normal
- Checks within max view distance
- Checks collisions TODO
*/
bool ray_views_point(const Eigen::Vector3f& vp_pose,
					const Eigen::Vector3f& vp_orient,
					const Eigen::Vector3f& ray_direction,
					const Eigen::Vector3f& surface_normal,
					const Eigen::Vector3f& point,
					double& max_incidence_angle,
					double& max_view_distance,
					double& camera_half_fov);

// From embree library. Find the closest point between a point and a triangle
Eigen::Vector3f closest_point_to_triangle(Eigen::Vector3f const& p, const Triangle& triangle);

float point_to_triangle_distance(const Eigen::Vector3f& point, const mesh_utils::Triangle& triangle);

float closest_distance_between_segments(
    const Eigen::Vector3f& a0, const Eigen::Vector3f& a1,
    const Eigen::Vector3f& b0, const Eigen::Vector3f& b1);

}