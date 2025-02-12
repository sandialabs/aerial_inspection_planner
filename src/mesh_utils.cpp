#include "aerial_inspection_planner/mesh_utils.hpp"

namespace mesh_utils
{

// from ChatGPT
bool ray_intersects_triangle(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_direction, const Triangle& triangle) {   
    //Möller–Trumbore intersection algorithm
    const float EPSILON = 1e-8f;
    Eigen::Vector3f V0 = triangle.vertices[0];
    Eigen::Vector3f V1 = triangle.vertices[1];
    Eigen::Vector3f V2 = triangle.vertices[2];
    Eigen::Vector3f E1 = V1 - V0;
    Eigen::Vector3f E2 = V2 - V0;
    Eigen::Vector3f P = ray_direction.cross(E2);
    float det = E1.dot(P);

    if (fabs(det) < EPSILON) {
        return false; // Ray is parallel to the triangle plane
    }

    float invDet = 1.0f / det;
    Eigen::Vector3f T = ray_origin - V0;
    float u = T.dot(P) * invDet;

    if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u-1) > EPSILON))
    {
        return false;  
    }    

    Eigen::Vector3f Q = T.cross(E1);
    float v = ray_direction.dot(Q) * invDet;

    if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
    {
        return false;
    }

    float t = E2.dot(Q) * invDet;

    return t > EPSILON; // Intersection is valid if t > 0
}

// from ChatGPT 
bool is_point_inside_mesh(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDirection, const std::vector<Triangle>& mesh) {
    int intersectionCount = 0;

    for (const auto& triangle : mesh) {
        if (ray_intersects_triangle(rayOrigin, rayDirection, triangle)) {
            intersectionCount++;
        }
    }

    return intersectionCount % 2 == 1;
}

bool segment_intersects_triangle(const Eigen::Vector3f& P0, const Eigen::Vector3f& P1, const Triangle& triangle) {
    const float EPSILON = 1e-8f;
    Eigen::Vector3f V0 = triangle.vertices[0];
    Eigen::Vector3f V1 = triangle.vertices[1];
    Eigen::Vector3f V2 = triangle.vertices[2];
    Eigen::Vector3f E1 = V1 - V0;
    Eigen::Vector3f E2 = V2 - V0;
    Eigen::Vector3f D = P1 - P0;
    Eigen::Vector3f P = D.cross(E2);
    float det = E1.dot(P);

    if (fabs(det) < EPSILON) {
        return false; // Segment is parallel to the triangle plane
    }

    float invDet = 1.0f / det;
    Eigen::Vector3f T = P0 - V0;
    float u = T.dot(P) * invDet;

    if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u-1) > EPSILON))
    {
        return false;  
    }    

    Eigen::Vector3f Q = T.cross(E1);
    float v = D.dot(Q) * invDet;

    if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
    {
        return false;
    }

    float t = E2.dot(Q) * invDet;

    if (t < 0 || t > 0.9999) {
        return false; // Intersection point is not within the segment
    }

    // Calculate the intersection point
    Eigen::Vector3f intersectionPoint = P0 + t * D;

    // Check if the intersection point is at any of the triangle's vertices
    if ((intersectionPoint - V0).norm() < EPSILON ||
        (intersectionPoint - V1).norm() < EPSILON ||
        (intersectionPoint - V2).norm() < EPSILON) {
        return false; // Intersection point is at a vertex
    }
    // RCLCPP_INFO(logger_, "segment ray is blocked with: %f length %f u %f v %f", t, u, v);
    return true;
}

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
    Eigen::Vector3f neg_ray_direction = ray_direction*-1;
	if (angle_between_vectors(surface_normal, neg_ray_direction) > max_incidence_angle)
	{     
        // point within incendent angle from surface normal
        Eigen::Vector3f neg_surface_normal = surface_normal*-1;
        if (angle_between_vectors(neg_surface_normal, neg_ray_direction) > max_incidence_angle)
        {
            return false;
        }
	}

	// Checks Max distance
    Eigen::Vector3f dist = vp_pose-point;
	if ((dist).norm() > max_view_distance)
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

// taken from ChatGPT
float closest_distance_between_segments(
    const Eigen::Vector3f& a0, const Eigen::Vector3f& a1,
    const Eigen::Vector3f& b0, const Eigen::Vector3f& b1)
{
    // Calculate vectors and their magnitudes
    Eigen::Vector3f A = a1 - a0;
    Eigen::Vector3f B = b1 - b0;
    Eigen::Vector3f r = a0 - b0;
    float a = A.dot(A); // Squared length of segment A
    float e = B.dot(B); // Squared length of segment B
    float f = B.dot(r);

    float s, t;
    float c = A.dot(r);
    float b = A.dot(B);
    float denom = a * e - b * b;

    // If segments are not parallel, compute the closest points
    if (denom != 0.0f) {
        s = (b * f - c * e) / denom;
        t = (a * f - b * c) / denom;

        // Clamp s to [0, 1]
        s = std::max(0.0f, std::min(1.0f, s));
    } else {
        // If segments are parallel, choose arbitrary s
        s = 0.0f;
    }

    // Compute the point on A closest to B
    Eigen::Vector3f c1 = a0 + s * A;

    // Compute the point on B closest to A
    t = (b * s + f) / e;

    // Clamp t to [0, 1]
    t = std::max(0.0f, std::min(1.0f, t));

    // Compute the point on B closest to A
    Eigen::Vector3f c2 = b0 + t * B;

    // Return the distance between the closest points
    return (c1 - c2).norm();
}

}