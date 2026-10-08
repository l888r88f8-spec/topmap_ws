cmake_minimum_required(VERSION 3.14)
project(topmap_ros)

# All C++ targets in this workspace are built as C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Reasonable strict warnings; third-party / generated headers are pulled in as
# SYSTEM includes so their diagnostics never fail our build (-Werror is not global).
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(topmap_core REQUIRED)
find_package(topmap_surface REQUIRED)
find_package(topmap_topology REQUIRED)
find_package(topmap_planner REQUIRED)
find_package(topmap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(topmap_ros
  src/type_conversions.cpp
  src/marker_builder.cpp
)
target_include_directories(topmap_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(topmap_ros PUBLIC cxx_std_17)
target_link_libraries(topmap_ros PUBLIC
  rclcpp::rclcpp
  topmap_core::topmap_core
  topmap_surface::topmap_surface
  topmap_topology::topmap_topology
  topmap_planner::topmap_planner
  Eigen3::Eigen
  ${topmap_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# Install the conversion/marker library and its public headers, then export the
# target so topmap_ros nodes and future packages can link it.
install(
  TARGETS topmap_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp topmap_core topmap_surface topmap_topology
                          topmap_planner topmap_msgs nav_msgs geometry_msgs
                          visualization_msgs Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_type_conversions test/test_type_conversions.cpp)
  if(TARGET test_type_conversions)
    target_link_libraries(test_type_conversions topmap_ros)
  endif()
  ament_add_gtest(test_marker_builder test/test_marker_builder.cpp)
  if(TARGET test_marker_builder)
    target_link_libraries(test_marker_builder topmap_ros)
  endif()
endif()

ament_package()
