cmake_minimum_required(VERSION 3.8)
project(voxel_map)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3 -fexceptions)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN -DMP_PROC_NUM=4)
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenMP QUIET)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  "msg/States.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(voxel_mapping_odom src/voxelMapping.cpp src/preprocess.cpp)
target_include_directories(voxel_mapping_odom PRIVATE
  include
  src
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(voxel_mapping_odom
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(voxel_mapping_odom OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(voxel_mapping_odom
  geometry_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  std_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(voxel_mapping_odom ${cpp_typesupport_target})

install(TARGETS voxel_mapping_odom DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME})

ament_package()
