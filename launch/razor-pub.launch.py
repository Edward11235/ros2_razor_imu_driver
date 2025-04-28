# Copyright (c) 2019, Andreas Klintberg
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for publishing only Razor IMU data (no display).
    """
    pkg = 'ros2_razor_imu'  # must match your package name
    config_path = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'my_razor.yaml'       # or 'razor.yaml' if that's your filename
    )

    imu_node = Node(
        package=pkg,
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([imu_node])
