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

add_library(topmap_core
  src/error.cpp
  src/params.cpp
)
target_include_directories(topmap_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(topmap_core PUBLIC cxx_std_17)
target_link_libraries(topmap_core PRIVATE yaml-cpp)

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
endif()

ament_package()
