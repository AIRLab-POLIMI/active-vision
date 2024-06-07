cmake_minimum_required(VERSION 3.8)
project(fruit_picking_pointcloud)


# Compiler

# Default to C99 (added)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 (added)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fruit_picking_interfaces REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)


include_directories(include)


# In order to add the executable that can be used by ros2
add_executable(segmented_pointcloud
  src/main_segmented_pointcloud.cpp
  src/segmented_pointcloud.cpp 

)
ament_target_dependencies(segmented_pointcloud
  rclcpp
  rclcpp_components
  depth_image_proc
  cv_bridge
  fruit_picking_interfaces
  PCL
  pcl_conversions
)


# In order to add the executable that can be used by ros2
add_executable(pointcloud
  src/main_pointcloud.cpp
)
ament_target_dependencies(pointcloud
  rclcpp
  rclcpp_components
  depth_image_proc
  cv_bridge
  fruit_picking_interfaces
  PCL
  pcl_conversions
)


# In order to add the library that can be used by ros2
set(library_name fruit_picking_pointcloud)
add_library(${library_name} SHARED
  src/segmented_pointcloud.cpp
)
ament_target_dependencies(${library_name}
  rclcpp
  rclcpp_components
  depth_image_proc
  cv_bridge
  fruit_picking_interfaces
  PCL
  pcl_conversions
)


# In order to let ros2 find the executable
install(TARGETS 
  pointcloud
	DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  segmented_pointcloud
	DESTINATION lib/${PROJECT_NAME}
)

# In order to let ros2 find the library
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# In order to let ros2 find the folders
install( 
  DIRECTORY include launch src
  DESTINATION share/${PROJECT_NAME}
)


# In order to let the library be visible from other packages
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()