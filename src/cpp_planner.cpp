#include <ament_index_cpp/get_package_share_directory.hpp>

#include "aerial_inspection_planner/cpp_planner.hpp"
#include "aerial_inspection_planner/mesh_utils.hpp"


InspectionPlanner::InspectionPlanner()
: Node("cpp_planner")
{
	mesh_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_mesh", 10);
	all_vps_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("all_vps", 10);
	vps_redundancy_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("all_vps_redundancy", 10);
	mesh_colored_as_views_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_mesh_colored_as_views", 10);
	vps_after_trim_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualize_vps_after_trimmed", 10);
	mesh_colored_as_views_after_trim_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_mesh_colored_as_views_after_trimmed", 10);
	final_path_visualization_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_final_path", 10);
	
	this->declare_parameter("visualize_mesh", true);
	this->declare_parameter("distance_from_surface", 4.0);
	this->declare_parameter("min_height", 0.3);
	this->declare_parameter("max_height", 999999.0);
	this->declare_parameter("min_view_redundancy", 2);
	this->declare_parameter("max_incidence_angle", 50.0);
	this->declare_parameter("max_view_distance", 10.0);
	this->declare_parameter("camera_half_fov", 55.0);
	this->declare_parameter("planner_type", "greedy");
	this->declare_parameter("min_distance", 1.5);
	this->declare_parameter("tsp_method", "greedy");
	this->declare_parameter("tsp_solver_allowed_time", 5.0);
	this->declare_parameter("mesh_name", "cargo_decimate");
	this->declare_parameter("rviz_scale", 1.0);
	this->declare_parameter("export_to_yaml", false);
	get_parameter("visualize_mesh", visualize_mesh_);
	get_parameter("distance_from_surface", distance_from_surface_);
	get_parameter("min_height", min_height_);
	get_parameter("max_height", max_height_);
	get_parameter("min_view_redundancy", min_view_redundancy_);
	get_parameter("max_incidence_angle", max_incidence_angle_);
	get_parameter("max_view_distance", max_view_distance_);
	get_parameter("camera_half_fov", camera_half_fov_);
	get_parameter("planner_type", planner_type_);
	get_parameter("min_distance", min_distance_);
	get_parameter("tsp_method", tsp_method_);
	get_parameter("tsp_solver_allowed_time", tsp_solver_allowed_time_);
	get_parameter("mesh_name", mesh_name_);
	get_parameter("rviz_scale", rviz_scale_);
	get_parameter("export_to_yaml", export_to_yaml_);
}

visualization_msgs::msg::Marker InspectionPlanner::create_mesh_visualization_msg(bool color_as_redundancy = false)
{
	visualization_msgs::msg::Marker marker;
	std_msgs::msg::ColorRGBA triangle_color;
	std_msgs::msg::ColorRGBA triangle_color2;
	std_msgs::msg::ColorRGBA triangle_color3;
	marker.header.frame_id = "map";
	marker.header.stamp = this->now();
	marker.ns = "cpp_planner";
	marker.id = 0;
	marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	triangle_color.a = 1.0;
	triangle_color.r = 0.0;
	triangle_color.g = 1.0;
	triangle_color.b = 0.0;	
	triangle_color2.a = 1.0;
	triangle_color2.r = 0.8;
	triangle_color2.g = 1.0;
	triangle_color2.b = 0.0;	
	triangle_color3.a = 1.0;
	triangle_color3.r = 0.0;
	triangle_color3.g = 0.5;
	triangle_color3.b = 0.5;
	double max_view_color = 1;
	double min_view_color = 9999999;
	if (color_as_redundancy)
	{
		for (size_t j = 0; j < mesh->size(); ++j)
		{
			int vp_count = mesh_to_vp_map[j].size();
			if (vp_count > max_view_color)
			{
				max_view_color = vp_count;
			}
			if (vp_count < min_view_color)
			{
				min_view_color = vp_count;
			}
		}
	}

	geometry_msgs::msg::Point pt;
	for (size_t j = 0; j < mesh->size(); ++j)
	{
		if (color_as_redundancy)
		{
			int vp_count = mesh_to_vp_map[j].size();
			if (vp_count == 0)
            {
                triangle_color.r = 0;
                triangle_color.g = 1;
                triangle_color.b = 0;
            }
            else
            {
                double view_mixer = std::min(vp_count, (int) max_view_color)/max_view_color;
                triangle_color.r = view_mixer;
                triangle_color.g = 0;
                triangle_color.b = 1 - view_mixer;
            }
			triangle_color2 = triangle_color;
			triangle_color3 = triangle_color;
		}
		const mesh_utils::Triangle& triangle = (*mesh)[j];
		pt.x = triangle.vertices[0].x();
		pt.y = triangle.vertices[0].y();
		pt.z = triangle.vertices[0].z();
		marker.points.push_back(pt);
		marker.colors.push_back(triangle_color);

		pt.x = triangle.vertices[1].x();
		pt.y = triangle.vertices[1].y();
		pt.z = triangle.vertices[1].z();
		marker.points.push_back(pt);
		marker.colors.push_back(triangle_color2);

		pt.x = triangle.vertices[2].x();
		pt.y = triangle.vertices[2].y();
		pt.z = triangle.vertices[2].z();
		marker.points.push_back(pt);
		marker.colors.push_back(triangle_color3);
	}
	return marker;
}

visualization_msgs::msg::Marker InspectionPlanner::create_path_visualization_msg(bool color_as_progress = false)
{
	visualization_msgs::msg::Marker marker;
	std_msgs::msg::ColorRGBA line_color;
	marker.header.frame_id = "map";
	marker.header.stamp = this->now();
	marker.ns = "cpp_planner";
	marker.id = 0;
	marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.5 * rviz_scale_;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;	
	line_color.a = 1.0;

	// Create and initialize start_color
    std_msgs::msg::ColorRGBA start_color;
    start_color.r = 0.0 / 255.0;  // Normalize to [0, 1]
    start_color.g = 255.0 / 255.0;
    start_color.b = 135.0 / 255.0;
    start_color.a = 1.0;  // Fully opaque

    // Create and initialize end_color
    std_msgs::msg::ColorRGBA end_color;
    end_color.r = 0.0 / 255.0;
    end_color.g = 97.0 / 255.0;
    end_color.b = 255.0 / 255.0;
    end_color.a = 1.0;  // Fully opaque

	double num_points = final_path.size();

	geometry_msgs::msg::Point pt;
	for (size_t j = 0; j < final_path.size(); ++j) 
	{
		if (color_as_progress)
		{
			line_color.r = (end_color.r-start_color.r)*j/num_points + start_color.r;
			line_color.g = (end_color.g-start_color.g)*j/num_points + start_color.g;
			line_color.b = (end_color.b-start_color.b)*j/num_points + start_color.b;
		}
		auto curr_point = vp_positions[final_path[j]];
		pt.x = curr_point.x();
		pt.y = curr_point.y();
		pt.z = curr_point.z();
		marker.points.push_back(pt);
		marker.colors.push_back(line_color);
	}
	return marker;
}

visualization_msgs::msg::MarkerArray InspectionPlanner::create_vps_visualization_msg(bool color_as_redundancy = false)
{
	visualization_msgs::msg::MarkerArray marker_array;
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = this->now();
	marker.ns = "cpp_planner";
	
	marker.type = visualization_msgs::msg::Marker::ARROW;
	marker.action = visualization_msgs::msg::Marker::ADD;
	if (color_as_redundancy)
	{
		marker.scale.x = 1.5 * rviz_scale_;
		marker.scale.y = .3 * rviz_scale_;
		marker.scale.z = .3 * rviz_scale_;
	}
	else
	{
		marker.scale.x = 1 * rviz_scale_;
		marker.scale.y = .1 * rviz_scale_;
		marker.scale.z = .1 * rviz_scale_;
	}
	
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;	
	double max_view_color = 1;
	double min_view_color = 9999999;
	if (color_as_redundancy)
	{
		for (auto i = 0u; i < vp_positions.size(); i++)
		{
			if (!vp_keep[i]) 
			{
				continue;
			}
			int vp_count = vp_to_mesh_map[i].size();
			if (vp_count > max_view_color)
			{
				max_view_color = vp_count;
			}
			if (vp_count < min_view_color)
			{
				min_view_color = vp_count;
			}
		}
	}

	Eigen::Quaternionf orientation;
	for (auto i = 0u; i < vp_positions.size(); i++) 
	{
		if (!vp_keep[i])
		{
			continue;
		}
		marker.id = i;
		marker.pose.position.x = vp_positions[i].x();
		marker.pose.position.y = vp_positions[i].y();
		marker.pose.position.z = vp_positions[i].z();
		orientation = mesh_utils::vector_to_quaternion(vp_orientation_vec[i]);
		marker.pose.orientation.x = orientation.x();
		marker.pose.orientation.y = orientation.y();
		marker.pose.orientation.z = orientation.z();
		marker.pose.orientation.w = orientation.w();
		if (color_as_redundancy)
		{
			int vp_count = vp_to_mesh_map[i].size();
			double view_mixer = std::min(vp_count, (int) max_view_color)/max_view_color;
			marker.color.r = view_mixer;
			marker.color.g = 0.0;
			marker.color.b = 1 - view_mixer;
		}
		marker_array.markers.push_back(marker);
	}
	return marker_array;
}

void InspectionPlanner::determine_visibility(bool save_mesh_to_vp_map = true)
{
    RCLCPP_INFO(this->get_logger(), "Calculating visibility for vps");
    auto start_time = this->now();
    // Resize the vectors before filling for faster execution
    mesh_to_vp_map.resize(mesh->size());
    vp_to_mesh_map.resize(vp_positions.size());

    // Determine visibility
    for (size_t i = 0; i < vp_positions.size(); ++i) // Viewpoints
    {
        const auto& vp_pose = vp_positions[i];
        const auto& vp_orient = vp_orientation_vec[i];
        // bool no_meshes_visible = true;
        // TODO Or already computed below? Just need to evaluate? Just look at all visible meshes and make sure they don't occlude each other?
        // actually still more complex. Need to look at all triangles, not just those passing the test
        // TODO triangles can be viewed even if the ray from vp to mesh passes through other meshes 
        for (size_t j = 0; j < mesh->size(); ++j) // Mesh triangles
        {
            const mesh_utils::Triangle& triangle = (*mesh)[j];
            bool all_corners_visible = true;
            for (size_t k = 0; k < triangle.vertices.size(); ++k) // Check all three corners
            {
                Eigen::Vector3f ray_direction(triangle.vertices[k] - vp_pose);
                ray_direction.normalize();
                if (!mesh_utils::ray_views_point(vp_pose, vp_orient, ray_direction, mesh_surface_normals[j],
                                    triangle.vertices[k], max_incidence_angle_, max_view_distance_,
                                    camera_half_fov_))
                {
                    all_corners_visible = false;
                    break;
                }
            }
            if (all_corners_visible) // Check if the segment from the viewpoint to the vertex intersects any other triangle
            {
                for (size_t k = 0; k < triangle.vertices.size(); ++k) // Check all three corners
                {   
                    for (size_t l = 0; l < mesh->size(); ++l) {
                        if (l == j) continue; // Skip the current triangle
                        // const mesh_utils::Triangle& other_triangle = (*mesh)[l]
                        if (mesh_utils::segment_intersects_triangle(vp_pose, triangle.vertices[k], (*mesh)[l])) {
                            all_corners_visible = false;
                            goto exitloops;
                        }
                    }
                }
            }
            exitloops:
            if (all_corners_visible)
            {
                // no_meshes_visible = false;
                if (save_mesh_to_vp_map)
                {
                    mesh_to_vp_map[j].push_back(i);
                }
                vp_to_mesh_map[i].push_back(j);
            }
        } 
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Time to calc visibility: " << this->now().seconds() - start_time.seconds()); 
}


bool InspectionPlanner::create_vps_for_all_triangles()
{
	RCLCPP_INFO(this->get_logger(), "**Creating all vps**");
	int in_collision_count = 0;
    int inside_mesh_count = 0;
    int occlusions_count = 0;
	for (std::vector<mesh_utils::Triangle>::iterator it = mesh->begin(); it != mesh->end(); it++)
	{
		int polygon_size = it->vertices.size();
		// RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Size is " << polygon_size);

		// Check that all polygons are triangles
		if (polygon_size != 3)
		{
			RCLCPP_ERROR(this->get_logger(), "Encountered polygon not of size 3");
			return false;
		}
		const Eigen::Vector3f v0(it->vertices[0].x(), it->vertices[0].y(), it->vertices[0].z());
		const Eigen::Vector3f v1(it->vertices[1].x(), it->vertices[1].y(), it->vertices[1].z());
		const Eigen::Vector3f v2(it->vertices[2].x(), it->vertices[2].y(), it->vertices[2].z()); 

		Eigen::Vector3f surface_normal = mesh_utils::calculate_surface_normal(v0, v1, v2);
		mesh_surface_normals.push_back(surface_normal);
		Eigen::Vector3f centroid = mesh_utils::calculate_centroid(v0, v1, v2);
        Eigen::Vector3f new_vp;
        for (int i = 0; i<2; i++)
        {
            surface_normal = (1-2*i)*surface_normal;
            // Find viewpoint out from mesh, set the viewing angle as opposite of the normal.
            new_vp = centroid + surface_normal*distance_from_surface_;

            // Check for collisions
            // TODO adjust vp if in collision. 
            if (is_collision(new_vp))
            {
                in_collision_count++;
                continue;
            }

            ///////////////// check for occlusions
            // TODO adjust vp if occluded
            bool all_corners_visible = true;
            for (size_t k = 0; k < it->vertices.size(); ++k) // Check all three corners
            {   
                for (size_t l = 0; l < mesh->size(); ++l) {
                    if (l == static_cast<size_t>(std::distance(mesh->begin(), it))) continue; // Skip the current triangle
                    // const mesh_utils::Triangle& other_triangle = (*mesh)[l]
                    if (mesh_utils::segment_intersects_triangle(new_vp, it->vertices[k], (*mesh)[l])) {
                        all_corners_visible = false;
                        goto exitloops;
                    }
                }
            }
            exitloops:
            if (!all_corners_visible)
            {
                occlusions_count++;
                continue;
            }

            ////////////////////// check for inside mesh
            if (mesh_utils::is_point_inside_mesh(new_vp, surface_normal, *mesh))
            {
                inside_mesh_count++;
                continue;
            }

            vp_positions.push_back(new_vp);
            vp_orientation_vec.push_back(-surface_normal); // pointing to surface normal, so negative
        }

	}
	vp_keep.resize(vp_positions.size(), true);
	if (in_collision_count > 0)
	{
		RCLCPP_WARN_STREAM(this->get_logger(), "Didn't create " 
			<< in_collision_count << " vps because of collisions");
	}
    if (occlusions_count > 0)
	{
		RCLCPP_WARN_STREAM(this->get_logger(), "Didn't create " 
			<< occlusions_count << " vps because of occlusions");
	}
    if (inside_mesh_count > 0)
	{
		RCLCPP_WARN_STREAM(this->get_logger(), "Didn't create " 
			<< inside_mesh_count << " vps because inside mesh");
	}
	return true;
}

bool InspectionPlanner::mesh_to_iterative_vps()
{
	RCLCPP_INFO(this->get_logger(), "Running Iterative VP creation");
	auto start_time = this->now();
	if (!mesh)
	{
		RCLCPP_ERROR(this->get_logger(), "Called VP sampling, but there is no mesh");
		return false;
	}

	// Create viewpoint for all triangles
	if (!create_vps_for_all_triangles())
	{
		return false;
	}
	
	if (this->visualize_mesh_)
	{
		visualization_msgs::msg::MarkerArray marker_array = this->create_vps_visualization_msg();
		// RCLCPP_INFO(this->get_logger(), "Publishing the vps!");
		all_vps_publisher_->publish(marker_array);
	}

	// Calculate visibility and view redundancy for each surface.
	determine_visibility();

    // check how many meshes can't be seen
    int mesh_not_seen_count = 0;
    for (size_t j = 0; j < mesh->size(); ++j)
	{
        int vp_count = mesh_to_vp_map[j].size();
        if (vp_count == 0)
        {
            mesh_not_seen_count++;
        }
    }
    RCLCPP_WARN(this->get_logger(), "%d Meshes not seen by view points before reduction", mesh_not_seen_count);

	// Visualize with redundancy as color. 
	if (this->visualize_mesh_)
	{
		visualization_msgs::msg::Marker marker = this->create_mesh_visualization_msg(true);
		// RCLCPP_INFO(this->get_logger(), "Publishing the mesh visualization with redundancy color!");
		mesh_colored_as_views_->publish(marker);
		visualization_msgs::msg::MarkerArray marker_array = this->create_vps_visualization_msg(true);
		// RCLCPP_INFO(this->get_logger(), "Publishing the vps with color based on number mesh seen!");
		vps_redundancy_publisher_->publish(marker_array);
	}

	// Iterate over list and remove as many as possible
	RCLCPP_INFO(this->get_logger(), "Removing Unnecessary Viewpoints");
	size_t vp_index = 0;
	int count_removed = 0;
	int count_vp_sees_nothing = 0;
	while (vp_index < vp_positions.size())
	{
		// Check if all triangles viewed by vp have above the min_view_redundancy, and can be removed
		bool can_remove = true;
		if (vp_to_mesh_map[vp_index].size() == 0)
		{
			vp_index++;
			count_vp_sees_nothing++;
			continue; // TODO, need better way. Each vp should be able to view its assigned mesh. 
			// Maybe have the vp move back from surface? Incident angle though?
		}
		for (size_t i = 0; i < vp_to_mesh_map[vp_index].size(); i++)
		{
			int mesh_index_i_of_vp = vp_to_mesh_map[vp_index][i];
			if (mesh_to_vp_map[mesh_index_i_of_vp].size() <= min_view_redundancy_)
			{
				vp_index++;
				can_remove = false;
				break;
			}
		}
		// TODO swap to pointers. For now just make a vp_keep vector. Will be faster too. 
		if (can_remove)
		{
			vp_keep[vp_index] = false;
			count_removed++;
			// Update the mesh to viewpoint map to not include this viewpoint
			for (size_t i = 0; i < vp_to_mesh_map[vp_index].size(); i++)
			{
				int mesh_index_i_of_vp = vp_to_mesh_map[vp_index][i];
				std::vector<int>::iterator position = std::find(
					mesh_to_vp_map[mesh_index_i_of_vp].begin(), 
					mesh_to_vp_map[mesh_index_i_of_vp].end(), 
					vp_index); // looks for vp_index 
				if (position != mesh_to_vp_map[mesh_index_i_of_vp].end())
				{
					mesh_to_vp_map[mesh_index_i_of_vp].erase(position);
				}
			}
			vp_index++; // TODO remove later when switch to pointers
		}
	}
	RCLCPP_INFO(this->get_logger(), "Finished Trimming");
	RCLCPP_INFO_STREAM(this->get_logger(), "Number VPs that can't see anything (and kept): " << count_vp_sees_nothing);
	RCLCPP_INFO_STREAM(this->get_logger(), "Number of VPs before and after: " 
		<< vp_positions.size() 
		<< " -> " 
		<< vp_positions.size() - count_removed);

    // check how many meshes can't be seen
    mesh_not_seen_count = 0;
    for (size_t j = 0; j < mesh->size(); ++j)
	{
        int vp_count = mesh_to_vp_map[j].size();
        if (vp_count == 0)
        {
            mesh_not_seen_count++;
        }
    }
    RCLCPP_WARN(this->get_logger(), "%d Meshes not seen by view points after reduction", mesh_not_seen_count);

	// Visualize the updates after trimming
	if (this->visualize_mesh_)
	{
		visualization_msgs::msg::Marker marker = this->create_mesh_visualization_msg(true);
		// RCLCPP_INFO(this->get_logger(), "Publishing the mesh visualization with removed vps!");
		mesh_colored_as_views_after_trim_->publish(marker);
		visualization_msgs::msg::MarkerArray marker_array = this->create_vps_visualization_msg(true);
		// RCLCPP_INFO(this->get_logger(), "Publishing the reduced number of vps!");
		vps_after_trim_->publish(marker_array);	
	}
	RCLCPP_INFO_STREAM(this->get_logger(), "Total time to solve: " << this->now().seconds() - start_time.seconds()); 

	return true;
}

bool InspectionPlanner::mesh_to_greedy_vps()
{
	RCLCPP_INFO(this->get_logger(), "Running Greedy VP Creation");
	auto start_time = this->now();
	if (!mesh)
	{
		RCLCPP_ERROR(this->get_logger(), "Called VP sampling, but there is no mesh! Exiting");
		return false;
	}

	// Create viewpoint for all triangles
	if (!create_vps_for_all_triangles())
	{
		return false;
	}
	
	if (this->visualize_mesh_)
	{
		visualization_msgs::msg::MarkerArray marker_array = this->create_vps_visualization_msg();
		RCLCPP_INFO(this->get_logger(), "Publishing the vps!");
		all_vps_publisher_->publish(marker_array);
	}

	// Calculate visibility and view redundancy for each surface.
	determine_visibility(true);
        
    // check how many meshes can't be seen
    int mesh_not_seen_count = 0;
    for (size_t j = 0; j < mesh->size(); ++j)
	{
        int vp_count = mesh_to_vp_map[j].size();
        if (vp_count == 0)
        {
            mesh_not_seen_count++;
        }
	}
    mesh_to_vp_map.clear();
    mesh_to_vp_map.resize(mesh->size());
    RCLCPP_WARN(this->get_logger(), "%d Meshes not seen by view points before reduction", mesh_not_seen_count);

	// Visualize with redundancy as color. 
	if (this->visualize_mesh_)
	{
		visualization_msgs::msg::Marker marker = this->create_mesh_visualization_msg(true);
		RCLCPP_DEBUG(this->get_logger(), "Publishing the mesh visualization with redundancy color!");
		mesh_colored_as_views_->publish(marker);
		visualization_msgs::msg::MarkerArray marker_array = this->create_vps_visualization_msg(true);
		RCLCPP_DEBUG(this->get_logger(), "Publishing the vps with color based on number mesh seen!");
		vps_redundancy_publisher_->publish(marker_array);
	}

	// Greedily keep nodes
	RCLCPP_INFO(this->get_logger(), "**Greedily selecting nodes**");
	vp_keep.resize(vp_positions.size(), false);
	std::fill(vp_keep.begin(), vp_keep.end(), false);
	int count_added = 0;
	int count_vp_sees_nothing = 0;

	std::vector<std::pair<int, int>> vp_to_value_pair;
	for (size_t i = 0; i < vp_to_mesh_map.size(); i++)
	{
		// If vp has a score of 0, add right away and don't add to sort pair list
		if (vp_to_mesh_map[i].size() == 0)
		{
			count_vp_sees_nothing++;
			count_added++;
			vp_keep[i] = true;
			continue; // TODO, need better way. Each vp should be able to view its assigned mesh. 
			// Maybe have the vp move back from surface? Incident angle though?
			// This shouldn't happen in the future. Means there isn't full coverage. But best to its
			// abilities. 
		}
		
		// Create new entry
		vp_to_value_pair.push_back(std::make_pair(i, vp_to_mesh_map[i].size()));
	}
	// Sort the vector of pairs based on value
	sort(vp_to_value_pair.begin(), vp_to_value_pair.end(), [=](std::pair<int, int>& a, std::pair<int, int>& b)
	{
		return a.second > b.second;
	}
	);

	RCLCPP_INFO_STREAM(this->get_logger(), "High vp index and value: " << vp_to_value_pair.front().first << ", " << vp_to_value_pair.front().second);
	RCLCPP_INFO_STREAM(this->get_logger(), "Low vp index and value: " << vp_to_value_pair.back().first << ", " << vp_to_value_pair.back().second);
    
    // Make copies of the maps
    auto vp_to_mesh_map_copy = vp_to_mesh_map;
    auto mesh_to_vp_map_copy = mesh_to_vp_map;
        
    // Initialize true maps to keep track of selected viewpoints and meshes
    std::vector<std::vector<int>> true_vp_to_mesh_map(vp_to_mesh_map.size());
    std::vector<std::vector<int>> true_mesh_to_vp_map(mesh_to_vp_map.size());

    // Main loop to greedily select viewpoints
    while (!vp_to_value_pair.empty()) {
        // Find the largest vp_to_value_pair
        auto max_it = std::max_element(vp_to_value_pair.begin(), vp_to_value_pair.end(),
                                    [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                                        return a.second < b.second;
                                    });

        // Check if the maximum element's second value is zero
        if (max_it->second == 0) {
            break;
        }

        // Commit it to vp_keep
        int top_vp_index = max_it->first;
        vp_keep[top_vp_index] = true;

        // Add the selected viewpoint and its meshes to the true maps
        true_vp_to_mesh_map[top_vp_index] = vp_to_mesh_map[top_vp_index];
        for (const auto& mesh : vp_to_mesh_map[top_vp_index]) {
            true_mesh_to_vp_map[mesh].push_back(top_vp_index);
        }

        // Remove it from vp_to_value_pair
        vp_to_value_pair.erase(max_it);


        // Update vp_to_mesh_map_copy and remove meshes seen by that vp if they meet min_view_redundancy_
        std::vector<int> meshes_to_remove = vp_to_mesh_map_copy[top_vp_index];
        for (int mesh : meshes_to_remove) {
            if (true_mesh_to_vp_map[mesh].size() >= min_view_redundancy_) {
                for (auto& vp_meshes : vp_to_mesh_map_copy) {
                    vp_meshes.erase(std::remove(vp_meshes.begin(), vp_meshes.end(), mesh), vp_meshes.end());
                }
            }
        }

        // Reset vp_to_value_pair
        vp_to_value_pair.clear();
        for (size_t i = 0; i < vp_to_mesh_map_copy.size(); i++) {
            if (!vp_keep[i]) {
                vp_to_value_pair.push_back(std::make_pair(i, vp_to_mesh_map_copy[i].size()));
            }
        }

        count_added++;
    }

    // Replace the original maps with the true maps
    vp_to_mesh_map = std::move(true_vp_to_mesh_map);
    mesh_to_vp_map = std::move(true_mesh_to_vp_map);

	RCLCPP_INFO(this->get_logger(), "Finished vp selection");
	RCLCPP_INFO_STREAM(this->get_logger(), "Number VPs that can't see anything (and kept): " << count_vp_sees_nothing);
	RCLCPP_INFO_STREAM(this->get_logger(), "Number of VPs before and after: " 
		<< vp_positions.size() 
		<< " -> " 
		<< count_added);

	// Visualize the updates after trimming
	if (this->visualize_mesh_)
	{
		visualization_msgs::msg::Marker marker = this->create_mesh_visualization_msg(true);
		RCLCPP_DEBUG(this->get_logger(), "Publishing the mesh visualization with removed vps!");
		mesh_colored_as_views_after_trim_->publish(marker);
		visualization_msgs::msg::MarkerArray marker_array = this->create_vps_visualization_msg(true);
		RCLCPP_DEBUG(this->get_logger(), "Publishing the reduced number of vps!");
		vps_after_trim_->publish(marker_array);	
	}
	RCLCPP_INFO_STREAM(this->get_logger(), "Total time to find vps: " << this->now().seconds() - start_time.seconds()); 
    
    // check how many meshes can't be seen
    mesh_not_seen_count = 0;
    for (size_t j = 0; j < mesh->size(); ++j)
	{
        int vp_count = mesh_to_vp_map[j].size();
        if (vp_count == 0)
        {
            mesh_not_seen_count++;
        }
	}
    RCLCPP_WARN(this->get_logger(), "%d Meshes not seen by view points after reduction", mesh_not_seen_count);

	return true;
}

bool InspectionPlanner::is_collision(Eigen::Vector3f point) const
{
	if (point.z() < min_height_ || point.z() > max_height_) // min_distance
	{
		return true;
	}
	
	for (size_t j = 0; j < mesh->size(); j++)
	{
		// Check endpoints for being close to the mesh
		const mesh_utils::Triangle& triangle = (*mesh)[j];
		if (mesh_utils::point_to_triangle_distance(point, triangle) < min_distance_)
		{
			return true;
		}
	}
	return false;
}

// Checks for collisions of a line segment between index vp_i and vp_j of the vp_positions
bool InspectionPlanner::is_collision(int vp_i, int vp_j) const
{
	// TODO RRT*, especially for nodes that have no connections
	// Could use BVH, Octree, or k-d tree to speed up, octomap TODO BVH if need to save time or later.
	Eigen::Vector3f first_state = vp_positions[vp_i];
	Eigen::Vector3f last_state = vp_positions[vp_j];

	// Check for collision with ground
	if (first_state.z() < min_height_ || last_state.z() < min_height_
		|| first_state.z() > max_height_ || last_state.z() > max_height_)
	{
		return true;
	}

	// Check mesh
	for (size_t j = 0; j < mesh->size(); j++)
	{
		const mesh_utils::Triangle& triangle = (*mesh)[j];
		// Could technically not check the endpoints because they are already checked
		if (mesh_utils::point_to_triangle_distance(first_state, triangle) < min_distance_ ||
			mesh_utils::point_to_triangle_distance(last_state, triangle) < min_distance_ ||
			mesh_utils::closest_distance_between_segments(first_state, last_state, triangle.vertices[0], triangle.vertices[1]) < min_distance_ ||
			mesh_utils::closest_distance_between_segments(first_state, last_state, triangle.vertices[1], triangle.vertices[2]) < min_distance_ ||
			mesh_utils::closest_distance_between_segments(first_state, last_state, triangle.vertices[2], triangle.vertices[0]) < min_distance_)
		{
			return true;
		}
	}
	return false;
}

bool InspectionPlanner::solve_tsp()
{
	// TODO fix bnb or find better solver. Greedy can allow collisions, and hc isn't able to fix sometimes
	RCLCPP_INFO(this->get_logger(), "Starting TSP");
	auto start_time = this->now();
	double collision_cost = 999999;
	// Create vector of kept vp indexes to map (TODO swap out when switch to pointers)
	std::vector<int> kept_vp_indexes;
	for (size_t i = 0; i < vp_keep.size(); i++)
	{
		if (vp_keep[i])
		{
			kept_vp_indexes.push_back(i);
		}
	}
	// Compute distance matrix
	std::vector<std::vector<double>> distance;
	distance.resize(kept_vp_indexes.size());
	for (size_t i = 0; i < kept_vp_indexes.size(); i++)
	{
		distance[i].resize(kept_vp_indexes.size());
		for (size_t j = 0; j <= i; j++) // Only iterate up to i
		{
			if (!is_collision(kept_vp_indexes[i], kept_vp_indexes[j]))
			{
				double dist = (vp_positions[kept_vp_indexes[i]] - vp_positions[kept_vp_indexes[j]]).norm();
				distance[i][j] = dist;
				distance[j][i] = dist;
			}
			else
			{
				distance[i][j] = collision_cost;
				distance[j][i] = collision_cost;
			}
		}
	}

	RCLCPP_INFO_STREAM(this->get_logger(), "Time to calc distances: " << this->now().seconds() - start_time.seconds()); 
	start_time = this->now();
	// check for the case where none can connect to a node
	std::vector<size_t> rows_to_remove;
	for (size_t i = 0; i < distance.size(); i++)
	{
		bool all_collision = true;
		for (size_t j = 0; j < distance.size(); j++) // Only iterate up to i
		{
			if (distance[i][j] < collision_cost)
			{
				all_collision = false;
				break;
			}
		}
		if (all_collision)
		{
			// remove from keep list and remove from distance matrix
			rows_to_remove.push_back(i);
			vp_keep[kept_vp_indexes[i]] = false;
		}
	}

	if (rows_to_remove.size() > 0)
	{
		RCLCPP_WARN_STREAM(this->get_logger(), "Removing " 
			<< rows_to_remove.size() << " vps because no vps connect to them in a straight line\n"
			<< "Could be due to geometry or vp being inside the mesh."); // TODO these viewpoints should have been made differently so this doesn't happen.
		
		// Remove rows from distance matrix
		for (size_t i = 0; i < rows_to_remove.size(); i++)
		{

			distance.erase(distance.begin() + rows_to_remove[i] - i);
			kept_vp_indexes.erase(kept_vp_indexes.begin() + rows_to_remove[i] - i);
		}
		// Remove columns from distance matrix
		for (size_t i = 0; i < rows_to_remove.size(); i++)
		{
			for (size_t j = 0; j < distance.size(); j++)
			{
				distance[j].erase(distance[j].begin() + rows_to_remove[i] - i);
			}
		}
	}
	
	// Solve TSP
	RCLCPP_INFO_STREAM(this->get_logger(), "TSP solver method: " << tsp_method_); 
	std::vector<int> tsp_index_order;
	if (tsp_method_ == "ortools")
	{
		std::vector<std::vector<int>> distance_int;
		double double_to_int_scale = 1000;
		distance_int.resize(distance.size());
		for (size_t i = 0; i < distance.size(); i++)
		{
			distance_int[i].resize(distance.size());
			for (size_t j = 0; j <= i; j++) // Only iterate up to i
			{
				distance_int[i][j] = distance[i][j] * double_to_int_scale;
				distance_int[j][i] = distance[j][i] * double_to_int_scale;
			}
		}
		tsp_index_order = operations_research::Tsp(distance_int, tsp_solver_allowed_time_, this->get_logger());
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "VP sampling failed");
		return false;
	}

	std::ostringstream oss;
	for (size_t i = 0; i < tsp_index_order.size(); i++)
	{
		oss << tsp_index_order[i] << ",";
	}
	RCLCPP_DEBUG_STREAM(this->get_logger(), oss.str());
	
	// Map indexes to true vp index and set 
	for (size_t i = 0; i < tsp_index_order.size(); i++)
	{
		final_path.push_back(kept_vp_indexes[tsp_index_order[i]]);
	}
	RCLCPP_INFO_STREAM(this->get_logger(), "Time to solve TSP: " << this->now().seconds() - start_time.seconds()); 

	// Visualize final path
	if (final_path.size() > 0 && this->visualize_mesh_)
	{
		visualization_msgs::msg::Marker marker = this->create_path_visualization_msg(true);
		RCLCPP_DEBUG(this->get_logger(), "Publishing the final path visualization");
		final_path_visualization_->publish(marker);
	}

	return true;
}

void InspectionPlanner::export_to_yaml()
{
	std::string file_name = ament_index_cpp::get_package_share_directory("aerial_inspection_planner") + "/inspection_path.yaml";
	std::fstream oss(file_name, std::ios::out);

	if (!oss.is_open()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("export_to_file"), "Failed to open file: " << file_name);
        return;
    }

	oss << "/drone_names: ['icarus']\n\n";
	oss << "# Icarus Drone\n/icarus:\n  properties:\n    use_sim: True\n  pose:";
	oss << "\n    initial:\n      position:\n        x: 2.5\n        y: -2.5\n        z: 2.0"; // TODO udpate initial position?? z position?
	oss << "\n      orientation:\n        roll: 0.0\n        pitch: 0.0\n        yaw: 0.0";
	oss << "\n  waypoints: [";

	for (size_t j = 0; j < final_path.size(); ++j)
	{
		Eigen::Vector3f curr_point = vp_positions[final_path[j]];
		Eigen::Vector3f curr_orient = vp_orientation_vec[final_path[j]];
		if (j != 0)
		{
			oss << ",\n              ";
		}
		double yaw = mesh_utils::vector_to_yaw(curr_orient);
		oss << "[" << curr_point.x() 
			<< ", " << curr_point.y() 
			<< ", " << curr_point.z()
			<< ", " << yaw 
			<< "]";
	}
	oss << "]";
	oss.close();
	RCLCPP_INFO_STREAM(this->get_logger(), "Finishing export to csv file: " << file_name);
}


void InspectionPlanner::plan()
{
	RCLCPP_WARN(this->get_logger(), "Starting New Plan!");
	auto start_time = this->now();
	// Process stl file to mesh
	RCLCPP_DEBUG(this->get_logger(), "Converting STL to Mesh");
	auto path = ament_index_cpp::get_package_share_directory("aerial_inspection_planner");
	RCLCPP_DEBUG_STREAM(this->get_logger(), path);
	this->mesh = mesh_utils::readSTLfile(
		ament_index_cpp::get_package_share_directory(
			"aerial_inspection_planner") + "/data/meshes/" + mesh_name_ + ".stl", this->get_logger());
	if (this->mesh->size() == 0)
	{
		RCLCPP_ERROR(this->get_logger(), "STL error. Exiting");
		return;
	}
	// Visualize mesh
	if (this->visualize_mesh_ && mesh)
	{
		visualization_msgs::msg::Marker marker = this->create_mesh_visualization_msg();
		RCLCPP_INFO(this->get_logger(), "Visualizing Mesh. Waiting for 3 seconds for Rviz to start...");
		rclcpp::sleep_for(std::chrono::seconds{3});
		// RCLCPP_INFO(this->get_logger(), "Publishing the mesh visualization!");
		mesh_publisher_->publish(marker);
	}

	// Process mesh file to VPs
	bool sampling_success = false;
	if (planner_type_ == "iterative")
	{
		sampling_success = this->mesh_to_iterative_vps();
	}
	else if (planner_type_ == "greedy")
	{
		sampling_success = this->mesh_to_greedy_vps();
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "planner_type doesn't match existing method");
		return;
	}
	
	if (!sampling_success)
	{
		RCLCPP_ERROR(this->get_logger(), "VP sampling failed");
		return;
	}


	// Process VPs to graph or directly optimize
	// Find path between vps
	RCLCPP_INFO(this->get_logger(), "**Solving for flight path**");
	bool success_tsp = this->solve_tsp(); 
	if (!success_tsp || final_path.size() == 0)
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), "TSP failed? Path size: " << final_path.size());
		return;
	}

	double path_length = 0;
	Eigen::Vector3f prev_vp_pos = vp_positions[final_path[0]];
	for (size_t j = 1; j < final_path.size(); ++j)
	{
		Eigen::Vector3f cur_vp_pos = vp_positions.at(final_path[j]);
		path_length += (prev_vp_pos - cur_vp_pos).norm();
		prev_vp_pos = cur_vp_pos;
	}
	RCLCPP_INFO_STREAM(this->get_logger(), "Final path length: " << path_length);
    if (path_length > 999999)
    {
        RCLCPP_WARN(this->get_logger(), "!Path has collision with object! A possible solution is to increase time to solve.");
    }
	
	RCLCPP_WARN(this->get_logger(), "Finished making plan!");
	RCLCPP_INFO_STREAM(this->get_logger(), "Total Time: " << this->now().seconds() - start_time.seconds()); 

	if (export_to_yaml_)
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "Exporting to CSV...");
		this->export_to_yaml();
	}

	return;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto planner_node = std::make_shared<InspectionPlanner>();
	planner_node->plan();
	// rclcpp::spin(planner_node);
	rclcpp::shutdown();
	return 0;
}