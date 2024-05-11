cmake_minimum_required(VERSION 3.8)
project(fruit_picking_octomap)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# added
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

# added
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

# added
add_library(fruit_picking_octomap SHARED
  src/extended_octomap_server.cpp
  src/extended_octomap_data.cpp
  src/octomap_server.cpp
  src/transforms.cpp
  src/conversions.cpp
  )

# added
ament_target_dependencies(fruit_picking_octomap
  rclcpp
  PCL
  pcl_conversions
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
  octomap
  octomap_msgs
  message_filters
  tf2_ros
  tf2_msgs
  tf2
  tf2_geometry_msgs
  rclcpp_components
  fruit_picking_interfaces
  )

# added
target_link_libraries(fruit_picking_octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

# added
rclcpp_components_register_node(fruit_picking_octomap
  PLUGIN "octomap_server::OctomapServer"
  EXECUTABLE octomap_server)

# added
rclcpp_components_register_node(fruit_picking_octomap
  PLUGIN "extended_octomap_server::ExtendedOctomapServer"
  EXECUTABLE extended_octomap_server)

#added
install(TARGETS
  fruit_picking_octomap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(
  DIRECTORY include launch src
  DESTINATION share/${PROJECT_NAME}
)

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
