#include "aerial_inspection_planner/bvh.hpp"

AABB BVH::computeBoundingBox(const mesh_utils::Triangle& triangle)
{
    Eigen::Vector3f minCorner = triangle.vertices[0];
    Eigen::Vector3f maxCorner = triangle.vertices[0];
    for (const auto& vertex : triangle.vertices)
    {
        minCorner = {std::min(minCorner.x(), vertex.x()), std::min(minCorner.y(), vertex.y()), std::min(minCorner.z(), vertex.z())};
        maxCorner = {std::max(maxCorner.x(), vertex.x()), std::max(maxCorner.y(), vertex.y()), std::max(maxCorner.z(), vertex.z())};
    }
    return {minCorner, maxCorner};
}

BVHNode* BVH::buildBVH(std::vector<mesh_utils::Triangle>& triangles, int start, int end)
{
    if (start == end)
    {
        return nullptr;
    }
    if (start + 1 == end)
    {
        BVHNode* node = new BVHNode();
        node->boundingBox = computeBoundingBox(triangles[start]);
        node->triangle = &triangles[start];
        node->left = node->right = nullptr;
        return node;
    }

    AABB boundingBox = computeBoundingBox(triangles[start]);
    for (int i = start + 1; i < end; ++i) 
    {
        AABB box = computeBoundingBox(triangles[i]);
        boundingBox.min = {std::min(boundingBox.min.x(), box.min.x()), std::min(boundingBox.min.y(), box.min.y()), std::min(boundingBox.min.z(), box.min.z())};
        boundingBox.max = {std::max(boundingBox.max.x(), box.max.x()), std::max(boundingBox.max.y(), box.max.y()), std::max(boundingBox.max.z(), box.max.z())};
    }

    int axis = 0;
    Eigen::Vector3f extents = boundingBox.max - boundingBox.min;
    if (extents.y() > extents.x()) axis = 1;
    if (extents.z() > extents[axis]) axis = 2;

    sort(triangles.begin() + start, triangles.begin() + end, [axis](const mesh_utils::Triangle& a, const mesh_utils::Triangle& b)
    {
        return a.vertices[0][axis] < b.vertices[0][axis];
    });

    int mid = (start + end) / 2;
    BVHNode* node = new BVHNode();
    node->boundingBox = boundingBox;
    node->left = buildBVH(triangles, start, mid);
    node->right = buildBVH(triangles, mid, end);
    node->triangle = nullptr;
    return node;
}
