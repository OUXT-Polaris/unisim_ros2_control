// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/qos.hpp>
#include <regex>
#include <unisim_ros2_control/unisim_ros2_control.hpp>

namespace unisim_ros2_control
{
UniSimRos2ControlComponent::UniSimRos2ControlComponent(const rclcpp::NodeOptions & options)
: Node("unisim_ros2_control", options)
{
  description_sub_ = create_subscription<std_msgs::msg::String>(
    "robot_description", RobotDescriptionQos(),
    std::bind(&UniSimRos2ControlComponent::robotDescriptionCallback, this, std::placeholders::_1));
}

void UniSimRos2ControlComponent::robotDescriptionCallback(
  const std_msgs::msg::String::SharedPtr description)
{
  std::string urdf_string = description->data;
  std::smatch mesh_tag_matches;
  std::cout << urdf_string << std::endl;
  if (std::regex_match(urdf_string, mesh_tag_matches, std::regex(R"(mesh)"))) {
    for (const auto & mesh_tag_match : mesh_tag_matches) {
      std::cout << mesh_tag_match.str() << std::endl;
    }
  }
  std::cout << mesh_tag_matches.size() << std::endl;
}
}  // namespace unisim_ros2_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(unisim_ros2_control::UniSimRos2ControlComponent)