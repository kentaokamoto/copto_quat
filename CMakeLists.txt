cmake_minimum_required(VERSION 3.5)
project(copto_quat)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(quat_component SHARED src/quat_component.cpp)

target_compile_definitions(
  quat_component 
  PRIVATE "COPTO_QUAT_QUAT_COMPONENT_BUILDING_DLL"
  )

ament_target_dependencies(quat_component 
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  geometry_msgs
  )

rclcpp_components_register_nodes(
  quat_component "copto_quat::QUATComponent"
  )

add_executable(copto_quat_node src/copto_quat_node.cpp)

ament_target_dependencies(copto_quat_node 
  rclcpp 
  rclcpp_components
  sensor_msgs 
  geometry_msgs
  )

target_link_libraries(copto_quat_node quat_component Eigen3::Eigen)

install(TARGETS copto_quat_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS quat_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_export_libraries(quat_component)

ament_export_include_directories(include)
ament_package()
