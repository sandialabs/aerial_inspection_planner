#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <algorithm>
#include <limits>
#include <cmath>
#include "aerial_inspection_planner/mesh_utils.hpp"



struct AABB {
    Eigen::Vector3f min;
    Eigen::Vector3f max;
};


struct BVHNode {
    AABB boundingBox;
    BVHNode* left;
    BVHNode* right;
    mesh_utils::Triangle* triangle;
};


class BVH
{
public:



private:
    AABB computeBoundingBox(const mesh_utils::Triangle& triangle);
    BVHNode* buildBVH(std::vector<mesh_utils::Triangle>& triangles, int start, int end);


};