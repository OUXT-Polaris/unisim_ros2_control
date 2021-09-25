# Copyright (c) 2020 OUXT Polaris
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

from posixpath import join
from ament_index_python.packages import get_package_share_directory
import os
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_urdf():
    share_dir_path = os.path.join(get_package_share_directory("unisim_ros2_control"))
    xacro_path = os.path.join(share_dir_path, "xacro", "whill_modelc.xacro")
    urdf_path = os.path.join(share_dir_path, "xacro", "whill_modelc.urdf")
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    return urdf_path

def generate_launch_description():

    return launch.LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                arguments=[generate_urdf()],
            ),
            Node(
                package="unisim_ros2_control",
                executable="unisim_ros2_control_node",
                output="screen",
            ),
        ]
    )
