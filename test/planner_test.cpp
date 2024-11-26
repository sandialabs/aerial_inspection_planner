#include <gtest/gtest.h>
#include "aerial_inspection_planner/cpp_planner.hpp"
#include "aerial_inspection_planner/mesh_utils.hpp"

TEST(package_name, a_first_test)
{
  ASSERT_EQ(4, 2 + 2);
  ASSERT_TRUE(true);
}

TEST(cpp_planner, point_to_triangle)
{
    double abs_error = 0.0001;
    mesh_utils::Triangle new_triangle;
    Eigen::Vector3f vertex = {0, 0, 0};
    vertex[0] = 0;
    vertex[1] = 0;
    vertex[2] = 0;
    new_triangle.vertices[0] = vertex;
    vertex[0] = 2;
    vertex[1] = 0;
    vertex[2] = 0;
    new_triangle.vertices[1] = vertex;
    vertex[0] = 0;
    vertex[1] = 2;
    vertex[2] = 0;
    new_triangle.vertices[2] = vertex;

    Eigen::Vector3f point = {2.0, 2.0, 1.0};
    Eigen::Vector3f closest_point = mesh_utils::closest_point_to_triangle(point, new_triangle);
    ASSERT_NEAR(closest_point.x(), 1, abs_error);
    ASSERT_NEAR(closest_point.y(), 1, abs_error);
    ASSERT_NEAR(closest_point.z(), 0, abs_error);

    point = {3.0, 0.0, 1.0};
    closest_point = mesh_utils::closest_point_to_triangle(point, new_triangle);
    ASSERT_NEAR(closest_point.x(), 2, abs_error);
    ASSERT_NEAR(closest_point.y(), 0, abs_error);
    ASSERT_NEAR(closest_point.z(), 0, abs_error);
}

TEST(cpp_planner, distance_between_segments)
{
    double abs_error = 0.0001;
    Eigen::Vector3f a0 = {0, 0, 0};
    Eigen::Vector3f a1 = {0, 2, 0};
    Eigen::Vector3f b0 = {-1, -1, 0};
    Eigen::Vector3f b1 = {1, -1, 0};
    float dist = mesh_utils::closest_distance_between_segments(a0, a1, b0, b1);
    ASSERT_NEAR(dist, 1, abs_error);
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}