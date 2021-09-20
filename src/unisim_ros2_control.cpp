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

#include <urdf_parser/urdf_parser.h>

// #include <regex>
#include <unisim_ros2_control/unisim_ros2_control.hpp>

namespace unisim_ros2_control
{
UniSimRos2ControlComponent::UniSimRos2ControlComponent(const rclcpp::NodeOptions & options)
: Node("unisim_ros2_control", options)
{
}

void UniSimRos2ControlComponent::robotDescriptionCallback(
  const std_msgs::msg::String::SharedPtr description)
{
  std::string urdf_string = description->data;
  const auto urdf = urdf::parseURDF(urdf_string);
  std::vector<urdf::LinkSharedPtr> links;
  urdf->getLinks(links);
  for (auto & link : links) {
    if (link->visual->geometry->type == urdf::Geometry::MESH) {
      std::cout << link->visual->material_name << std::endl;
    }
  }
  // std::regex re(R"(<mesh filename="*")");
  // std::regex re(R"("package://*")");
}
}  // namespace unisim_ros2_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(unisim_ros2_control::UniSimRos2ControlComponent)