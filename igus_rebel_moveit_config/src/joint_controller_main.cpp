/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <igus_rebel_moveit_config/joint_position_controller.hpp>
#include <igus_rebel_moveit_config/joint_trajectory_controller.hpp>

int main(int argc, char* argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_controller");
    // variable
    std::vector<std::string> joint_names;
    std::vector<std::string> ign_joint_topics;
    int update_rate;
    // parameters
    ros_node->declare_parameter("joint_names", joint_names);
    ros_node->declare_parameter("ign_joint_topics", ign_joint_topics);
    ros_node->declare_parameter("rate", 200);
    joint_names = ros_node->get_parameter("joint_names").as_string_array();
    ign_joint_topics = ros_node->get_parameter("ign_joint_topics").as_string_array();
    update_rate = ros_node->get_parameter("rate").as_int();
    // create controller
    auto joint_trajectory_controller = std::make_shared<igus_rebel_moveit_config::jointTrajectoryController>(ros_node,
        joint_names, "set_joint_trajectory", ign_joint_topics ,update_rate);
    // create controller 
    auto joint_position_controller = std::make_shared<igus_rebel_moveit_config::JointPositionController>(ros_node,
        joint_names, "joint_states", ign_joint_topics);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
