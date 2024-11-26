cmake_minimum_required(VERSION 3.8)
project(aerial_inspection_planner)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the path to the OR-Tools directory. **Update file path if located elsewhere**
set(ORTOOLS_DIR "/or-tools_x86_64_Ubuntu-22.04_cpp_v9.10.4067")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${ORTOOLS_DIR})
include_directories(${ORTOOLS_DIR}/include)
link_directories(${ORTOOLS_DIR}/lib)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

add_executable(cpp_planner src/cpp_planner.cpp src/bvh.cpp src/mesh_utils.cpp)
ament_target_dependencies(cpp_planner rclcpp std_msgs nav_msgs visualization_msgs Eigen3)
target_link_libraries(cpp_planner ortools protobuf::libprotobuf)
target_include_directories(cpp_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ORTOOLS_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  $<INSTALL_INTERFACE:include>)
target_compile_features(cpp_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  cpp_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  data
  launch
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/planner_test.cpp src/mesh_utils.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs nav_msgs visualization_msgs Eigen3)
  target_link_libraries(${PROJECT_NAME}_test ortools protobuf::libprotobuf)
endif()

ament_package()
