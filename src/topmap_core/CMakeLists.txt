cmake_minimum_required(VERSION 3.14)
project(topmap_core)

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
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(topmap_core
  src/error.cpp
  src/params.cpp
  src/binary_io.cpp
  src/geometry.cpp
)
target_include_directories(topmap_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(topmap_core PUBLIC cxx_std_17)
target_link_libraries(topmap_core PUBLIC Eigen3::Eigen)
target_link_libraries(topmap_core PRIVATE yaml-cpp)

# Install the library and its public headers, then export the target so that
# downstream ament packages can find_package(topmap_core) and link it.
install(
  TARGETS topmap_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake Eigen3 yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_error test/test_error.cpp)
  if(TARGET test_error)
    target_link_libraries(test_error topmap_core)
  endif()
  ament_add_gtest(test_params test/test_params.cpp)
  if(TARGET test_params)
    target_link_libraries(test_params topmap_core)
    set_tests_properties(test_params PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
  ament_add_gtest(test_binary_io test/test_binary_io.cpp)
  if(TARGET test_binary_io)
    target_link_libraries(test_binary_io topmap_core)
  endif()
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  if(TARGET test_geometry)
    target_link_libraries(test_geometry topmap_core)
  endif()
endif()

ament_package()
