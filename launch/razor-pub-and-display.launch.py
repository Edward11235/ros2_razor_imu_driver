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
    pkg  = 'ros2_razor_imu'   # ← make sure this matches your <package> name in package.xml
    cfg  = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'my_razor.yaml'      # ← or "razor.yaml" if that’s what you actually named it
    )

    imu_node = Node(
        package=pkg,
        executable='imu_node',             # ← your console_scripts entry-point
        name='imu_node',
        output='screen',
        parameters=[cfg],
    )

    display_node = Node(
        package=pkg,
        executable='display_3D_visualization_node',  # ← your entry-point for the viz script
        name='display_3D_visualization_node',
        output='screen',
    )

    return LaunchDescription([
        imu_node,
        display_node,
    ])

def main(args=None):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)
