cmake_minimum_required(VERSION 3.14)
project(topmap_planner)

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
find_package(topmap_surface REQUIRED)
find_package(topmap_topology REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(topmap_planner
  src/region_locator.cpp
  src/topology_astar.cpp
  src/corridor_builder.cpp
  src/surface_astar.cpp
  src/path_optimizer.cpp
  src/safety_validator.cpp
)
target_include_directories(topmap_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(topmap_planner PUBLIC cxx_std_17)
target_link_libraries(topmap_planner PUBLIC
  topmap_core::topmap_core
  topmap_surface::topmap_surface
  topmap_topology::topmap_topology
  Eigen3::Eigen)

# Install the library and its public headers, then export the target so that
# downstream ament packages can find_package(topmap_planner) and link it.
install(
  TARGETS topmap_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake topmap_core topmap_surface topmap_topology Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_region_locator test/test_region_locator.cpp)
  if(TARGET test_region_locator)
    target_link_libraries(test_region_locator topmap_planner)
  endif()
  ament_add_gtest(test_topology_astar test/test_topology_astar.cpp)
  if(TARGET test_topology_astar)
    target_link_libraries(test_topology_astar topmap_planner)
  endif()
  ament_add_gtest(test_corridor_builder test/test_corridor_builder.cpp)
  if(TARGET test_corridor_builder)
    target_link_libraries(test_corridor_builder topmap_planner)
  endif()
  ament_add_gtest(test_surface_astar test/test_surface_astar.cpp)
  if(TARGET test_surface_astar)
    target_link_libraries(test_surface_astar topmap_planner)
  endif()
  ament_add_gtest(test_path_optimizer test/test_path_optimizer.cpp)
  if(TARGET test_path_optimizer)
    target_link_libraries(test_path_optimizer topmap_planner)
  endif()
  ament_add_gtest(test_safety_validator test/test_safety_validator.cpp)
  if(TARGET test_safety_validator)
    target_link_libraries(test_safety_validator topmap_planner)
    # This suite builds several full synthetic maps and intentionally resamples
    # a long off-surface segment; it exceeds ament's 60 s default on Debug ARM
    # builds even though every individual case completes promptly.
    set_tests_properties(test_safety_validator PROPERTIES TIMEOUT 180)
  endif()
endif()

ament_package()
