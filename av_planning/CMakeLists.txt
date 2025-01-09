cmake_minimum_required(VERSION 3.8)
project(av_planning)


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
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(mobile_manipulation_interfaces REQUIRED)
find_package(igus_rebel_gripper_controller REQUIRED)
find_package(moveit2_api REQUIRED)
find_package(OpenMP REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(av_interfaces REQUIRED)
find_package(av_octomap REQUIRED)
find_package(av_pointcloud REQUIRED)
find_package(depth_image_proc REQUIRED)

include_directories(include)



# In order to add the executable that can be used by ros2
add_executable(main_predefined_planning_pipeline src/main_predefined_planning_pipeline.cpp src/predefined_planning_pipeline.cpp)
ament_target_dependencies(main_predefined_planning_pipeline
  rclcpp
  rclcpp_components
  PCL
  pcl_conversions
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  geometry_msgs
  cv_bridge
  pluginlib
  moveit_core
  moveit_kinematics
  moveit_ros_planning_interface
  moveit_visual_tools
  mobile_manipulation_interfaces
  igus_rebel_gripper_controller
  moveit2_api
  OPENMP
  message_filters
  octomap
  octomap_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  rclcpp_components
  av_interfaces
  av_octomap
  av_pointcloud
  depth_image_proc
  )

# In order to let ros2 find the executable
install(TARGETS 
  main_predefined_planning_pipeline
	DESTINATION lib/${PROJECT_NAME}
)

# In order to add the executable that can be used by ros2
add_executable(main_nbv_planning_pipeline src/main_nbv_planning_pipeline.cpp src/nbv_planning_pipeline.cpp)
ament_target_dependencies(main_nbv_planning_pipeline
  rclcpp
  rclcpp_components
  PCL
  pcl_conversions
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  geometry_msgs
  cv_bridge
  pluginlib
  moveit_core
  moveit_kinematics
  moveit_ros_planning_interface
  moveit_visual_tools
  mobile_manipulation_interfaces
  igus_rebel_gripper_controller
  moveit2_api
  OPENMP
  message_filters
  octomap
  octomap_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  rclcpp_components
  av_interfaces
  av_octomap
  av_pointcloud
  depth_image_proc
  )
if(OpenMP_CXX_FOUND)
  target_link_libraries(main_nbv_planning_pipeline OpenMP::OpenMP_CXX)
endif()

# In order to let ros2 find the executable
install(TARGETS 
  main_nbv_planning_pipeline
	DESTINATION lib/${PROJECT_NAME}
)

# In order to let ros2 find the folders
install( 
  DIRECTORY include launch src config
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
