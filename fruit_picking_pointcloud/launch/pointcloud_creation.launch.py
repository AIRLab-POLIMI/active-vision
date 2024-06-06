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
    
    depth_image_topic_arg = DeclareLaunchArgument(
        name="depth_image_topic",
        default_value="/depth_image",
        description="Topic containing the depth image to convert",
    )

    rgb_image_topic_arg = DeclareLaunchArgument(
        name="rgb_image_topic",
        default_value="/rgb_image",
        description="Topic containing the rgb image to convert",
    )

    depth_image_camera_info_topic_arg = DeclareLaunchArgument(
        name="depth_image_camera_info_topic",
        default_value="/depth_image_camera_info",
        description="Topic containing the input depth image camera info",
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        name="pointcloud_topic",
        default_value="/pointcloud",
        description="Topic containing the pointcloud data created combining input depth image and rgb image",
    )

    return LaunchDescription([
        
        depth_image_topic_arg,
        rgb_image_topic_arg,
        depth_image_camera_info_topic_arg,
        pointcloud_topic_arg,

        Node(
            package='fruit_picking_pointcloud',
            executable='pointcloud',
            output='screen',
            arguments= [
                "--ros-args",
                "--log-level",
                "pointcloud:=debug",
            ],
            remappings=[('depth_registered/image_rect', LaunchConfiguration('depth_image_topic')),
                        ('rgb/image_rect_color', LaunchConfiguration('rgb_image_topic')),
                        ('rgb/camera_info', LaunchConfiguration('depth_image_camera_info_topic')),
                        ('points', LaunchConfiguration('pointcloud_topic'))],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
