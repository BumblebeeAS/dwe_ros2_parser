# Copyright 2024 - Urlaxle
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

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    # Get package share directory
    pkg_dir = get_package_share_directory('dwe_ros2_parser')
    
    # Construct path to calibration file
    calib_file = os.path.join(pkg_dir, 'config', 'dwe_calib.yaml')

    # Setup project paths
    dwe_parser = Node(
        package='dwe_ros2_parser',
        executable='dwe_ros2_parser',
        parameters=[{
          'device': "/dev/dwe_camera",
          'image_topic': '/dwe/image_raw',
          'camera_info_topic': '/dwe/camera_info',
          'calib_file': calib_file,
          'width': 800,
          'height': 600,
          'framerate': 12,
          'auto_exposure': False,
          'exposure': 100,
          'show_image' : False, 
          'use_h264': False,
          'save_images': False,
          'save_folder': '/home/gg/camera',
          'image_prefix': 'front',
        }],
        output='screen'
    )

    return LaunchDescription([
        dwe_parser,
    ])