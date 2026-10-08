cmake_minimum_required(VERSION 3.14)
project(topmap_topology)

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
find_package(Eigen3 REQUIRED)

add_library(topmap_topology
  src/region_growing.cpp
  src/region_split.cpp
)
target_include_directories(topmap_topology PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(topmap_topology PUBLIC cxx_std_17)
target_link_libraries(topmap_topology PUBLIC
  topmap_core::topmap_core
  topmap_surface::topmap_surface
  Eigen3::Eigen)

# Install the library and its public headers, then export the target so that
# downstream ament packages can find_package(topmap_topology) and link it.
install(
  TARGETS topmap_topology
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake topmap_core topmap_surface Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_region_growing test/test_region_growing.cpp)
  if(TARGET test_region_growing)
    target_link_libraries(test_region_growing topmap_topology)
  endif()
  ament_add_gtest(test_region_split test/test_region_split.cpp)
  if(TARGET test_region_split)
    target_link_libraries(test_region_split topmap_topology)
  endif()
endif()

ament_package()
