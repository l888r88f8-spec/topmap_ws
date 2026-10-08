cmake_minimum_required(VERSION 3.14)
project(topmap_msgs)

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
find_package(rosidl_default_generators REQUIRED)
# rosidl requires every interface dependency to be found before generation.
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Region.msg"
  "msg/Portal.msg"
  "msg/TopologyGraph.msg"
  "msg/TopologyPath.msg"
  "msg/MapStats.msg"
  "msg/PlanStats.msg"
  "srv/BuildMap.srv"
  "srv/PlanPath.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_msgs_roundtrip test/test_msgs_roundtrip.cpp)
  if(TARGET test_msgs_roundtrip)
    target_link_libraries(test_msgs_roundtrip ${PROJECT_NAME}__rosidl_typesupport_cpp)
  endif()
endif()

ament_package()
