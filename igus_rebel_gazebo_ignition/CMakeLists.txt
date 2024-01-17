cmake_minimum_required(VERSION 3.8)
project(igus_rebel_gazebo_ignition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(igus_rebel_description_ros2 REQUIRED)
find_package(igus_rebel_gazebo_ignition REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

# include
include_directories(include)

# generate universal_robot_ign lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    std_msgs
    trajectory_msgs
    sensor_msgs
)


add_executable(joint_controller src/joint_controller_main.cpp)
target_link_libraries(joint_controller ${PROJECT_NAME})
add_executable(joint_state_publisher src/joint_state_publisher_main.cpp)
target_link_libraries(joint_state_publisher ${PROJECT_NAME})


target_link_libraries(${PROJECT_NAME}
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

install(
  DIRECTORY config include launch src urdf
  DESTINATION share/${PROJECT_NAME}
)


# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
    
# Install include directories
install(DIRECTORY include/
  DESTINATION include
)


# Install executables
install(TARGETS joint_controller joint_state_publisher
  DESTINATION lib/${PROJECT_NAME} 
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