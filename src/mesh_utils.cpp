#include "aerial_inspection_planner/mesh_utils.hpp"

namespace mesh_utils
{

// From ATL inspection stack with modifications
std::unique_ptr<std::vector<Triangle>> readSTLfile(std::string name, rclcpp::Logger logger_)
{
	std::vector<nav_msgs::msg::Path> mesh;
	auto inspection_area = std::make_unique<std::vector<Triangle>>();
	std::fstream f;
	f.open(name.c_str());
	assert(f.is_open());
	int MaxLine = 0;
	char* line = nullptr;
	double maxX = -DBL_MAX;
	double maxY = -DBL_MAX;
	double maxZ = -DBL_MAX;
	double minX = DBL_MAX;
	double minY = DBL_MAX;
	double minZ = DBL_MAX;
	line = (char *) malloc(MaxLine = 80);
	assert(line != nullptr);
	f.getline(line, MaxLine);
	//   (line) ? RCLCPP_ERROR(logger_, "Getline failed") : RCLCPP_INFO(logger_, "Read first line");
	assert(line);
	RCLCPP_INFO(logger_, "Opened file and read first line");
	char* token = strtok(line, " ");
	if (token == NULL)
	{
		std::cout << "Line: " << line << std::endl << std::endl;
		RCLCPP_ERROR(logger_, "Issue reading first line");
		rclcpp::shutdown();
		return inspection_area;
	}
	if (0 != strcmp(token, "solid"))
	{
		RCLCPP_ERROR(logger_, "Invalid mesh file! Make sure the file is given in ascii-format.");
		rclcpp::shutdown();
	}
	RCLCPP_INFO(logger_, "Mesh file is valid");
	line = (char *) malloc(MaxLine = 80);
	assert(line != nullptr);
	f.getline(line, MaxLine);
	int k = 0;
	while (0 != strcmp(strtok(line, " "), "endsolid")) //  && !ros::isShuttingDown()
	{
		int q = 0;
		nav_msgs::msg::Path p;
		geometry_msgs::msg::PoseStamped v1;
		for (int i = 0; i<7; i++)
		{
			while (line[q] == ' ')
			{
				q++;
			}
			if (line[q] == 'v')
			{
				const double yawTrafo = 0.0;      // used to rotate the mesh before processing
				const double scaleFactor = 1.0;   // used to scale the mesh before processing
				const double offsetX = 0.0;       // used to offset the mesh before processing
				const double offsetY = 0.0;       // used to offset the mesh before processing
				const double offsetZ = 0.0;       // used to offset the mesh before processing

				geometry_msgs::msg::PoseStamped vert;
				char* v = strtok(line+q," ");
				v = strtok(NULL," ");
				double xtmp = atof(v)/scaleFactor;
				v = strtok(NULL," ");
				double ytmp = atof(v)/scaleFactor;
				vert.pose.position.x = cos(yawTrafo)*xtmp-sin(yawTrafo)*ytmp;
				vert.pose.position.y =  sin(yawTrafo)*xtmp+cos(yawTrafo)*ytmp;
				v = strtok(NULL," ");
				vert.pose.position.z =  atof(v)/scaleFactor;
				vert.pose.position.x -= offsetX;
				vert.pose.position.y -= offsetY;
				vert.pose.position.z -= offsetZ;
				if(maxX<vert.pose.position.x)
					maxX=vert.pose.position.x;
				if(maxY<vert.pose.position.y)
					maxY=vert.pose.position.y;
				if(maxZ<vert.pose.position.z)
					maxZ=vert.pose.position.z;
				if(minX>vert.pose.position.x)
					minX=vert.pose.position.x;
				if(minY>vert.pose.position.y)
					minY=vert.pose.position.y;
				if(minZ>vert.pose.position.z)
					minZ=vert.pose.position.z;
				vert.pose.orientation.x =  0.0;
				vert.pose.orientation.y =  0.0;
				vert.pose.orientation.z =  0.0;
				vert.pose.orientation.w =  1.0;
				p.poses.push_back(vert);
				if(p.poses.size() == 1)
					v1 = vert;
			}
			line = (char *) malloc(MaxLine = 80);
			assert(line != nullptr);
			f.getline(line, MaxLine);
		}
		p.poses.push_back(v1);
		p.header.frame_id = "/kopt_frame";
		// p.header.stamp = ros::Time::now();
		mesh.push_back(p);
		k++;
	}
	free(line);
	f.close();
	RCLCPP_INFO(logger_, "Mesh area is bounded by: [%2.2f,%2.2f]x[%2.2f,%2.2f]x[%2.2f,%2.2f]", minX,maxX,minY,maxY,minZ,maxZ);
	
	for (std::vector<nav_msgs::msg::Path>::iterator it = mesh.begin(); it != mesh.end(); it++)
	{
		Triangle new_triangle;
		Eigen::Vector3f vertex;
		vertex[0] = it->poses[0].pose.position.x;
		vertex[1] = it->poses[0].pose.position.y;
		vertex[2] = it->poses[0].pose.position.z;
		new_triangle.vertices[0] = vertex;
		vertex[0] = it->poses[1].pose.position.x;
		vertex[1] = it->poses[1].pose.position.y;
		vertex[2] = it->poses[1].pose.position.z;
		new_triangle.vertices[1] = vertex;
		vertex[0] = it->poses[2].pose.position.x;
		vertex[1] = it->poses[2].pose.position.y;
		vertex[2] = it->poses[2].pose.position.z;
		new_triangle.vertices[2] = vertex;
		inspection_area->push_back(new_triangle);
	}
	RCLCPP_INFO_STREAM(logger_, "Mesh has " << inspection_area->size() << " triangles");
	return inspection_area;
}

Eigen::Vector3f calculate_surface_normal(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    Eigen::Vector3f edge1 = v1 - v0;
    Eigen::Vector3f edge2 = v2 - v0;
    Eigen::Vector3f normal = edge1.cross(edge2);
    normal.normalize();
    return normal;
}

Eigen::Vector3f calculate_centroid(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    return (v0 + v1 + v2) / 3.0f;
}

Eigen::Quaternionf vector_to_quaternion(const Eigen::Vector3f& unit_vector)
{
    // Define the reference direction (z-axis)
    Eigen::Vector3f reference_direction(1.0, 0.0, 0.0);

    // Create a quaternion and set it from the two vectors
    Eigen::Quaternionf quaternion;
    quaternion.setFromTwoVectors(reference_direction, unit_vector);

    return quaternion;
}

double vector_to_yaw(const Eigen::Vector3f& unit_vector)
{
    // Ensure the vector is normalized
    Eigen::Vector3f normalized_vector = unit_vector.normalized();

    // Calculate the yaw angle (rotation around the z-axis)
    double yaw = std::atan2(normalized_vector.y(), normalized_vector.x());

    return yaw;
}

// Function to compute the angle between two vectors in degrees
float angle_between_vectors(const Eigen::Vector3f& u, const Eigen::Vector3f& v)
{
    // Compute the dot product
    float dot_product = u.dot(v);

    // Compute the norms of the vectors
    float norm_u = u.norm();
    float norm_v = v.norm();

    // Compute the cosine of the angle
    float cos_angle = dot_product / (norm_u * norm_v);

    // Clamp the cosine to avoid numerical issues with acos
    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));

    // Compute the angle in radians
    float angle_radians = std::acos(cos_angle);

    // Convert the angle to degrees
    float angle_degrees = angle_radians * 180.0f / M_PI;

    return angle_degrees;
}

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
                                double& camera_half_fov) 
{
    // point within fov
	if (angle_between_vectors(vp_orient, ray_direction) > camera_half_fov)
	{
		return false;
	}

	// point within incendent angle from surface normal
	if (angle_between_vectors(surface_normal, -ray_direction) > max_incidence_angle)
	{
		return false;
	}

	// Checks Max distance
	if ((vp_pose-point).norm() > max_view_distance)
	{
		return false;
	}
	return true;
}

// From embree library. Find the closest point between a point and a triangle
Eigen::Vector3f closest_point_to_triangle(Eigen::Vector3f const& p, const Triangle& triangle)
{
	const Eigen::Vector3f& a = triangle.vertices[0];
	const Eigen::Vector3f& b = triangle.vertices[1];
	const Eigen::Vector3f& c = triangle.vertices[2];
    const Eigen::Vector3f ab = b - a;
    const Eigen::Vector3f ac = c - a;
    const Eigen::Vector3f ap = p - a;

    const float d1 = ab.dot(ap);
    const float d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f) return a;

    const Eigen::Vector3f bp = p - b;
    const float d3 = ab.dot(bp);
    const float d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3) return b;

    const Eigen::Vector3f cp = p - c;
    const float d5 = ab.dot(cp);
    const float d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6) return c;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        const float v = d1 / (d1 - d3);
        return a + v * ab;
    }
    
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        const float v = d2 / (d2 - d6);
        return a + v * ac;
    }
    
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + v * (c - b);
    }

    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + v * ab + w * ac;
}

float point_to_triangle_distance(const Eigen::Vector3f& point, const mesh_utils::Triangle& triangle) {
    Eigen::Vector3f closest_point = mesh_utils::closest_point_to_triangle(point, triangle);
    return (closest_point - point).norm();
}

// Overloaded function with default arguments
float closest_distance_between_segments(
    const Eigen::Vector3f& a0, const Eigen::Vector3f& a1,
    const Eigen::Vector3f& b0, const Eigen::Vector3f& b1) {
    return closest_distance_between_segments(a0, a1, b0, b1, true, false, false, false, false);
}

// From https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments
float closest_distance_between_segments(
    const Eigen::Vector3f& a0, const Eigen::Vector3f& a1,
    const Eigen::Vector3f& b0, const Eigen::Vector3f& b1,
    bool clampAll = true, bool clampA0 = false, bool clampA1 = false,
    bool clampB0 = false, bool clampB1 = false)
{
    // If clampAll is true, set all clamps to true
    if (clampAll)
    {
        clampA0 = true;
        clampA1 = true;
        clampB0 = true;
        clampB1 = true;
    }

    // Calculate vectors and their magnitudes
    Eigen::Vector3f A = a1 - a0;
    Eigen::Vector3f B = b1 - b0;
    float magA = A.norm();
    float magB = B.norm();

    Eigen::Vector3f _A = A / magA;
    Eigen::Vector3f _B = B / magB;

    Eigen::Vector3f cross = _A.cross(_B);
    float denom = cross.squaredNorm();

    // Define a small epsilon for floating-point comparison
    const float epsilon = 1e-6;

    // If lines are parallel (denom is close to 0), test if lines overlap
    if (denom < epsilon)
    {
        float d0 = _A.dot(b0 - a0);

        // Overlap only possible with clamping
        if (clampA0 || clampA1 || clampB0 || clampB1)
        {
            float d1 = _A.dot(b1 - a0);

            // Is segment B before A?
            if (d0 <= 0 && d1 <= 0)
            {
                if (clampA0 && clampB1)
                {
                    if (std::abs(d0) < std::abs(d1))
                    {
                        return (a0 - b0).norm();
                    }
                    return (a0 - b1).norm();
                }
            }
            // Is segment B after A?
            else if (d0 >= magA && d1 >= magA)
            {
                if (clampA1 && clampB0) 
                {
                    if (std::abs(d0) < std::abs(d1))
                    {
                        return (a1 - b0).norm();
                    }
                    return (a1 - b1).norm();
                }
            }
        }
        // Segments overlap, return distance between parallel segments
        return ((d0 * _A) + a0 - b0).norm();
    }

    // Lines criss-cross: Calculate the projected closest points
    Eigen::Vector3f t = b0 - a0;
    float detA = t.dot(_B.cross(cross));
    float detB = t.dot(_A.cross(cross));

    float t0 = detA / denom;
    float t1 = detB / denom;

    Eigen::Vector3f pA = a0 + (_A * t0); // Projected closest point on segment A
    Eigen::Vector3f pB = b0 + (_B * t1); // Projected closest point on segment B

    // Clamp projections
    if (clampA0 || clampA1 || clampB0 || clampB1)
    {
        if (clampA0 && t0 < 0)
        {
            pA = a0;
        }
        else if (clampA1 && t0 > magA)
        {
            pA = a1;
        }

        if (clampB0 && t1 < 0)
        {
            pB = b0;
        }
        else if (clampB1 && t1 > magB)
        {
            pB = b1;
        }

        // Clamp projection A
        if ((clampA0 && t0 < 0) || (clampA1 && t0 > magA))
        {
            float dot = _B.dot(pA - b0);
            if (clampB0 && dot < 0)
            {
                dot = 0;
            }
            else if (clampB1 && dot > magB)
            {
                dot = magB;
            }
            pB = b0 + (_B * dot);
        }

        // Clamp projection B
        if ((clampB0 && t1 < 0) || (clampB1 && t1 > magB))
        {
            float dot = _A.dot(pB - a0);
            if (clampA0 && dot < 0)
            {
                dot = 0;
            }
            else if (clampA1 && dot > magA)
            {
                dot = magA;
            }
            pA = a0 + (_A * dot);
        }
    }

    return (pA - pB).norm();
}

}