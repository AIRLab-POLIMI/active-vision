# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument



def generate_launch_description():

    rgb_image_topic_arg = DeclareLaunchArgument(
        name="rgb_image_topic",
        default_value="/rgb_image",
        description="Topic containing the input rgb image",
    )

    rgb_images_array_topic_arg = DeclareLaunchArgument(
        name="rgb_images_array_topic",
        default_value="/rgb_images_array",
        description="Topic containing the input rgb images array",
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        name="depth_image_topic",
        default_value="/depth_image",
        description="Topic containing the input depth image",
    )

    depth_image_camera_info_topic_arg = DeclareLaunchArgument(
        name="depth_image_camera_info_topic",
        default_value="/depth_image_camera_info",
        description="Topic containing the input depth image camera info",
    )

    segmented_pointcloud_topic_arg = DeclareLaunchArgument(
        name="segmented_pointcloud_topic",
        default_value="/segmented_pointcloud",
        description="Topic containing the output segmented pointcloud",
    )

    segmented_pointclouds_array_topic_arg = DeclareLaunchArgument(
        name="segmented_pointclouds_array_topic",
        default_value="/segmented_pointclouds_array",
        description="Topic containing the output segmented pointclouds array",
    )

    confidences_topic_arg = DeclareLaunchArgument(
        name="confidences_topic",
        default_value="/confidences",
        description="Topic containing the confidence of the rgb images array",
    )

    publish_pointclouds_array_arg = DeclareLaunchArgument(
        name="publish_pointclouds_array",
        default_value="False",
        choices=["True", "False"],
        description="Whether or not publishing the output segmented pointclouds array",
    )

    publish_single_pointcloud_arg = DeclareLaunchArgument(
        name="publish_single_pointcloud",
        default_value="False",
        choices=["True", "False"],
        description="Whether or not publishing the output segmented pointcloud",
    ) 



    return LaunchDescription([
        
        rgb_image_topic_arg,
        rgb_images_array_topic_arg,
        depth_image_topic_arg,
        depth_image_camera_info_topic_arg,
        segmented_pointcloud_topic_arg,
        segmented_pointclouds_array_topic_arg,
        confidences_topic_arg,
        publish_pointclouds_array_arg,
        publish_single_pointcloud_arg,
        

        Node(
            package='fruit_picking_pointcloud',
            executable='segmented_pointcloud',
            output='screen',
            arguments= [
                "--ros-args",
                "--log-level",
                "segmented_pointcloud:=debug",
            ],
            remappings=[('rgb_image', LaunchConfiguration('rgb_image_topic')),
                        ('rgb_images_array', LaunchConfiguration('rgb_images_array_topic')),
                        ('depth_image', LaunchConfiguration('depth_image_topic')),
                        ('depth_image_camera_info', LaunchConfiguration('depth_image_camera_info_topic')),
                        ('segmented_pointcloud', LaunchConfiguration('segmented_pointcloud_topic')),
                        ('segmented_pointclouds_array', LaunchConfiguration('segmented_pointclouds_array_topic')),
                        ('confidences', LaunchConfiguration('confidences_topic'))],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'publish_pointclouds_array': LaunchConfiguration('publish_pointclouds_array'),
                        'publish_single_pointcloud': LaunchConfiguration('publish_single_pointcloud')
                    }],
    )])
