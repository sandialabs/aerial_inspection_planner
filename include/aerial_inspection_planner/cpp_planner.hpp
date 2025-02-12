#pragma once
#include <algorithm>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "aerial_inspection_planner/bvh.hpp"
#include "tsp_solvers/ortools.hpp"

class InspectionPlanner : public rclcpp::Node
{
public:
	InspectionPlanner();
	~InspectionPlanner(){
		RCLCPP_INFO(this->get_logger(), "cpp_planner node destructor called");
	}
	void plan();
	

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_colored_as_views_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_colored_as_views_after_trim_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr final_path_visualization_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr all_vps_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vps_redundancy_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vps_after_trim_;
	
	// Set by ros param config
	bool visualize_mesh_; 
	double distance_from_surface_; 
	double min_height_;
	double max_height_;
	size_t min_view_redundancy_;
    double max_incidence_angle_;
    double max_view_distance_;
	double min_distance_;
	double camera_half_fov_;
	std::string planner_type_;
	std::string tsp_method_;
	double tsp_solver_allowed_time_;
	std::string mesh_name_;
	double rviz_scale_;
	bool export_to_yaml_;

	std::unique_ptr<std::vector<mesh_utils::Triangle>> mesh; // triangle mesh
	std::vector<Eigen::Vector3f> vp_positions;
	std::vector<Eigen::Vector3f> vp_orientation_vec; 
	std::vector<Eigen::Vector3f> mesh_surface_normals;
    std::vector<std::vector<int>> mesh_to_vp_map;
	std::vector<std::vector<int>> vp_to_mesh_map;
	std::vector<bool> vp_keep;
	std::vector<int> final_path;
	
	bool mesh_to_iterative_vps();
	bool mesh_to_greedy_vps();
	bool solve_tsp();
	visualization_msgs::msg::Marker create_mesh_visualization_msg(bool color_as_redundancy);
	visualization_msgs::msg::MarkerArray create_vps_visualization_msg(bool color_as_redundancy);
	visualization_msgs::msg::Marker create_path_visualization_msg(bool color_as_progress);
	void determine_visibility(bool save_mesh_to_vp_map);
	bool create_vps_for_all_triangles();
	bool is_collision(int vp_i, int vp_j) const;
	bool is_collision(Eigen::Vector3f point) const;
	void export_to_yaml();
};