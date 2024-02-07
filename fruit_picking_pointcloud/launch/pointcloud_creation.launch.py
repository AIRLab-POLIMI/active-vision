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

import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    
    depth_image_topic_arg = DeclareLaunchArgument(
        name="depth_image_topic",
        default_value="/virtual_camera_link/rgbd_camera/depth_image",
        description="Topic containing the depth image data to convert",
    )

    rgb_image_topic_arg = DeclareLaunchArgument(
        name="rgb_image_topic",
        default_value="/virtual_camera_link/rgbd_camera/image_raw",
        description="Topic containing the rgb image data to convert",
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        name="camera_info_topic",
        default_value="/virtual_camera_link/rgbd_camera/camera_info",
        description="Topic containing the camera info of the depth image and rgb image topics",
    )

    pointcloud_processed_topic_arg = DeclareLaunchArgument(
        name="pointcloud_processed_topic",
        default_value="/fruit_picking/pointcloud/pointcloud_processed",
        description="Topic containing the pointcloud data created combining input depth image and rgb image",
    )

    return LaunchDescription([
        
        depth_image_topic_arg,
        rgb_image_topic_arg,
        camera_info_topic_arg,
        pointcloud_processed_topic_arg,

        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                                ('rgb/image_rect_color', LaunchConfiguration('rgb_image_topic')),
                                ('depth_registered/image_rect', LaunchConfiguration('depth_image_topic')),
                                ('points', LaunchConfiguration('pointcloud_processed_topic'))],
                    # parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local',
                    #         }],
                ),
            ],
            output='screen',
        ),
    ])
