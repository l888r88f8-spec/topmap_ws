cmake_minimum_required(VERSION 3.14)
project(topmap_bringup)

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

# T32: topmap_bringup is a pure integration package. The algorithm YAML, the
# launch files and the RViz layout are the deliverable; there is no library or
# executable to build. Scripts are installed with the source executable bit so
# the acceptance entry point can be run directly.
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

ament_package()
