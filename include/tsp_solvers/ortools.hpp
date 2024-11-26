// Derived from example https://developers.google.com/optimization/routing/tsp#c++
#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace operations_research {
struct DataModel {
  std::vector<std::vector<int>> distance_matrix;
  const int num_vehicles = 1;
  const RoutingIndexManager::NodeIndex depot{0};
};

//! @brief Print the solution.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void PrintSolution(const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution,
                   rclcpp::Logger logger_) {
  // Inspect solution.
  RCLCPP_INFO_STREAM(logger_, "Objective: " << solution.ObjectiveValue() << " miles");
  int64_t index = routing.Start(0);
  RCLCPP_INFO_STREAM(logger_, "Route:");
  int64_t distance{0};
  std::stringstream route;
  while (!routing.IsEnd(index)) {
    route << manager.IndexToNode(index).value() << " -> ";
    const int64_t previous_index = index;
    index = solution.Value(routing.NextVar(index));
    distance += routing.GetArcCostForVehicle(previous_index, index, int64_t{0});
  }
  // RCLCPP_INFO_STREAM(logger_, route.str() << manager.IndexToNode(index).value());
  RCLCPP_INFO_STREAM(logger_, "Route cost: " << distance << " m");
  // RCLCPP_INFO_STREAM(logger_, "");
  // RCLCPP_INFO_STREAM(logger_, "Advanced usage:");
  // RCLCPP_INFO_STREAM(logger_, "Problem solved in " << routing.solver()->wall_time() << "ms");
}

std::vector<int> Tsp(std::vector<std::vector<int>> &distance, double planning_time, rclcpp::Logger logger_) {
  RCLCPP_INFO(logger_, "Starting the ortools tsp solver");
  // Instantiate the data problem.
  DataModel data;
  data.distance_matrix = distance;

  // Create Routing Index Manager
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);

  // Create Routing Model.
  RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](const int64_t from_index,
                        const int64_t to_index) -> int64_t {
        // Convert from routing variable Index to distance matrix NodeIndex.
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_local_search_metaheuristic(
    LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
    // Other options https://developers.google.com/optimization/routing/routing_options#local_search_options
    searchParameters.mutable_time_limit()->set_seconds(planning_time);
    // searchParameters.set_log_search(true);
    
//   searchParameters.set_first_solution_strategy(
//       FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  // Solve the problem.
  const Assignment* solution = routing.SolveWithParameters(searchParameters);

  // Print solution on console.
  PrintSolution(manager, routing, *solution, logger_);
  std::vector<int> solution_order;
  int64_t index = routing.Start(0);
  while (!routing.IsEnd(index)) {
    solution_order.push_back(manager.IndexToNode(index).value());
    index = solution->Value(routing.NextVar(index));
  }
  return solution_order;
}

}  // namespace operations_research

// int main(int /*argc*/, char* /*argv*/[]) {
//   operations_research::Tsp();
//   return EXIT_SUCCESS;
// }