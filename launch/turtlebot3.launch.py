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

from ament_index_python.packages import get_package_share_directory
import os
import launch
from launch_ros.actions import Node


def generate_robot_description(model="burger"):
    urdf_file_name = "turtlebot3_" + model + ".urdf"
    urdf = os.path.join(
        get_package_share_directory("turtlebot3_description"), "urdf", urdf_file_name
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()
    rsp_params = {"robot_description": robot_desc}


def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[generate_launch_description()],
            )
        ]
    )
