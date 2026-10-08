cmake_minimum_required(VERSION 3.14)
project(topmap_surface)

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
find_package(topmap_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features search kdtree)
find_package(CGAL REQUIRED)

add_library(topmap_surface
  src/point_cloud_processor.cpp
  src/surface_features.cpp
  src/mesh_types.cpp
  src/triangulator.cpp
  src/traversability.cpp
  src/mesh_serialization.cpp
  src/synthetic_scene.cpp
)
target_include_directories(topmap_surface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(topmap_surface SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_features(topmap_surface PUBLIC cxx_std_17)
target_link_libraries(topmap_surface PUBLIC topmap_core::topmap_core Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_options(topmap_surface PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(topmap_surface PRIVATE CGAL::CGAL)

# Install the library and its public headers, then export the target so that
# downstream ament packages can find_package(topmap_surface) and link it.
install(
  TARGETS topmap_surface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# R18: acceptance scripts invoke this through `ros2 run topmap_surface ...`, so
# the executable must be installed into the package lib directory.
add_executable(write_synthetic_pcd tools/write_synthetic_pcd.cpp)
target_link_libraries(write_synthetic_pcd topmap_surface)
install(TARGETS write_synthetic_pcd DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake topmap_core Eigen3 PCL CGAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_point_cloud_processor test/test_point_cloud_processor.cpp)
  if(TARGET test_point_cloud_processor)
    target_link_libraries(test_point_cloud_processor topmap_surface)
  endif()
  ament_add_gtest(test_surface_features test/test_surface_features.cpp)
  if(TARGET test_surface_features)
    target_link_libraries(test_surface_features topmap_surface)
  endif()
  ament_add_gtest(test_triangle_mesh test/test_triangle_mesh.cpp)
  if(TARGET test_triangle_mesh)
    target_link_libraries(test_triangle_mesh topmap_surface)
  endif()
  ament_add_gtest(test_triangulator test/test_triangulator.cpp)
  if(TARGET test_triangulator)
    target_link_libraries(test_triangulator topmap_surface)
  endif()
  ament_add_gtest(test_traversability test/test_traversability.cpp)
  if(TARGET test_traversability)
    target_link_libraries(test_traversability topmap_surface)
  endif()
  ament_add_gtest(test_mesh_serialization test/test_mesh_serialization.cpp)
  if(TARGET test_mesh_serialization)
    target_link_libraries(test_mesh_serialization topmap_surface)
  endif()
  ament_add_gtest(test_synthetic_scene test/test_synthetic_scene.cpp)
  if(TARGET test_synthetic_scene)
    target_link_libraries(test_synthetic_scene topmap_surface)
  endif()
endif()

ament_package()
