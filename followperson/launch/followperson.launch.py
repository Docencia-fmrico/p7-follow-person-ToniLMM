# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    yolo_bringup_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('yolo_bringup'),
            'launch',
            'yolov8.launch.py'
        )
    ),
    launch_arguments={
        'input_image_topic': '/rgbd_camera/image',
        'model': 'yolov8m-seg.pt',
        'image_reliability': '0',
    }.items()
    )


    followperson_node = Node(
        package='followperson',
        executable='followperson_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    detector_cmd = Node(package='camera',
        executable='yolo_detection',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
          ('input_detection', '/rgbd_camera/image'),
          ('camera_info', '/rgbd_camera/camera_info'),
          ('output_detection_2d', 'detection_2d'),
        ])

    convert_2d_3d = Node(package='camera',
        executable='detection_2d_to_3d_depth',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
          ('input_depth', '/rgbd_camera/depth_image'),
          ('input_detection_2d', 'detection_2d'),
          ('camera_info', '/rgbd_camera/camera_info'),
          ('output_detection_3d', 'detection_3d'),
        ])

    ld = LaunchDescription()
    ld.add_action(followperson_node)
    ld.add_action(detector_cmd)
    ld.add_action(convert_2d_3d)
    ld.add_action(yolo_bringup_launch)

    return ld
