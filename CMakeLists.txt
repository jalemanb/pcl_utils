cmake_minimum_required(VERSION 3.8)
project(pcl_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)

# Set the include directory
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add the include directory for all targets
include_directories(${INCLUDE_DIR})

# Composable Node to Concatenate two pointclouds into a given frame
add_library(pcl_utils_concatenator SHARED src/pointcloud_concatenator.cpp)
target_include_directories(pcl_utils_concatenator PUBLIC ${INCLUDE_DIR})
target_compile_definitions(pcl_utils_concatenator PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pcl_utils_concatenator  rclcpp pcl_ros pcl_conversions message_filters tf2_ros)
rclcpp_components_register_node(pcl_utils_concatenator PLUGIN "pcl_utils::PointCloudConcatenator" EXECUTABLE pcl_utils_concatenator_node)

# Composable Node to Crop a predefined box w.r.t. a given frame
add_library(pcl_utils_cropbox SHARED src/crop_box.cpp )
target_include_directories(pcl_utils_cropbox PUBLIC ${INCLUDE_DIR})
target_compile_definitions(pcl_utils_cropbox PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pcl_utils_cropbox  rclcpp pcl_ros pcl_conversions tf2_ros)
rclcpp_components_register_node(pcl_utils_cropbox PLUGIN "pcl_utils::CropBox" EXECUTABLE pcl_utils_cropbox_node)

# Composable Node to Filter a pointcloud based on 2 field of views
add_library(pcl_utils_fov_filter SHARED src/fov_filter.cpp )
target_include_directories(pcl_utils_fov_filter PUBLIC ${INCLUDE_DIR})
target_compile_definitions(pcl_utils_fov_filter PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pcl_utils_fov_filter  rclcpp pcl_ros pcl_conversions tf2_ros)
rclcpp_components_register_node(pcl_utils_fov_filter PLUGIN "pcl_utils::FOVFilter" EXECUTABLE pcl_utils_fov_filter_node)

# Composable Node Downsample a Pointcloud based on a VoxelGrid
add_library(pcl_utils_voxelgrid_filter SHARED src/voxelgrid_filter.cpp )
target_include_directories(pcl_utils_voxelgrid_filter PUBLIC ${INCLUDE_DIR})
target_compile_definitions(pcl_utils_voxelgrid_filter PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pcl_utils_voxelgrid_filter  rclcpp pcl_ros pcl_conversions tf2_ros)
rclcpp_components_register_node(pcl_utils_voxelgrid_filter PLUGIN "pcl_utils::VoxelGridFilter" EXECUTABLE pcl_utils_voxelgrid_filter_node)


install(TARGETS pcl_utils_concatenator pcl_utils_cropbox pcl_utils_fov_filter pcl_utils_voxelgrid_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
