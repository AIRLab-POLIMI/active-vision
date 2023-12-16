cmake_minimum_required(VERSION 3.8)
project(igus_rebel_moveit_config)

find_package(ament_cmake REQUIRED)

# ADDED
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(igus_rebel_description_ros2 REQUIRED)
find_package(igus_rebel_moveit_config REQUIRED)

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
  DIRECTORY include src
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



# END ADDED



install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()





