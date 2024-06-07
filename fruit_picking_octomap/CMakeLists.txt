cmake_minimum_required(VERSION 3.8)
project(fruit_picking_octomap)


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
find_package(PCL 1.10 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(fruit_picking_interfaces REQUIRED)


include_directories(include)


# In order to add the executable that can be used by ros2
add_executable(extended_octomap_server
	src/main_extended_octomap_server.cpp
	src/extended_octomap_server.cpp
	src/extended_octomap_data.cpp
)
ament_target_dependencies(extended_octomap_server
  rclcpp rclcpp_components
  PCL pcl_conversions
  sensor_msgs std_msgs nav_msgs visualization_msgs geometry_msgs
  std_srvs
  octomap octomap_msgs
  message_filters
  tf2_ros tf2_msgs tf2 tf2_geometry_msgs
  fruit_picking_interfaces
)

# In order to add the library that can be used by ros2
set(library_name fruit_picking_octomap)
add_library(${library_name} SHARED
  src/extended_octomap_server.cpp
  src/extended_octomap_data.cpp
)
ament_target_dependencies(${library_name}
  rclcpp rclcpp_components
  PCL pcl_conversions
  sensor_msgs std_msgs nav_msgs visualization_msgs geometry_msgs
  std_srvs
  octomap octomap_msgs
  message_filters
  tf2_ros tf2_msgs tf2 tf2_geometry_msgs
  fruit_picking_interfaces
)




# In order to let ros2 find the executable
install(TARGETS 
  extended_octomap_server
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


# In order to link some needed libraries into this package
target_link_libraries(extended_octomap_server
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_link_libraries(${library_name}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
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
