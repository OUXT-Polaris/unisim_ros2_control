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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <rclcpp/qos.hpp>
#include <sstream>
#include <unisim_ros2_control/unisim_ros2_control.hpp>
namespace unisim_ros2_control
{
UniSimRos2ControlComponent::UniSimRos2ControlComponent(const rclcpp::NodeOptions & options)
: Node("unisim_ros2_control", options)
{
  declare_parameter<std::string>("urdf_output_directory", "/tmp/unisim_ros2_control");
  urdf_output_directory_ = get_parameter("urdf_output_directory").as_string();
  makeDirectory(urdf_output_directory_);
  description_sub_ = create_subscription<std_msgs::msg::String>(
    "robot_description", RobotDescriptionQos(),
    std::bind(&UniSimRos2ControlComponent::robotDescriptionCallback, this, std::placeholders::_1));
}

std::string UniSimRos2ControlComponent::resolvePath(std::string filename)
{
  if (filename.length() <= 10) {
    const auto error =
      std::runtime_error("filename should be relative path from package, filename = " + filename);
    RCLCPP_ERROR(get_logger(), error.what());
    throw error;
  }
  if (filename.substr(0, 10) != "package://") {
    const auto error =
      std::runtime_error("filename should be relative path from package, filename = " + filename);
    RCLCPP_ERROR(get_logger(), error.what());
    throw error;
  }
  std::string rest = filename.substr(10, filename.length() - 1);
  std::list<std::string> string_list;
  boost::split(string_list, rest, boost::is_any_of("/"));
  std::string full_path;
  for (auto itr = string_list.begin(); itr != string_list.end(); itr++) {
    if (itr == string_list.begin()) {
      full_path = ament_index_cpp::get_package_share_directory(*itr);
    } else {
      full_path = full_path + "/" + *itr;
    }
  }
  return full_path;
}

void UniSimRos2ControlComponent::makeDirectory(const std::string & path)
{
  boost::filesystem::path directory(path);
  if (boost::filesystem::exists(directory)) {
    boost::filesystem::remove_all(path);
  }
  boost::system::error_code error;
  const bool result = boost::filesystem::create_directory(path, error);
  if (!result || error) {
    const auto error = std::runtime_error("failed to create directory at " + path);
    RCLCPP_ERROR(get_logger(), error.what());
    throw error;
  }
}

void UniSimRos2ControlComponent::copyModelFile(const std::string & from, const std::string & to)
{
  const boost::filesystem::path from_path(from);
  const boost::filesystem::path to_path(to);
  boost::filesystem::copy_file(from, to);
}

std::string UniSimRos2ControlComponent::getExtension(const std::string & path)
{
  return boost::filesystem::path(path).extension().generic_string();
}

void UniSimRos2ControlComponent::robotDescriptionCallback(
  const std_msgs::msg::String::SharedPtr description)
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_string(description->data.c_str());
  if (!result) {
    const auto error = std::runtime_error("failed to parse xml, xml = \n" + description->data);
    RCLCPP_ERROR(get_logger(), error.what());
    throw error;
  }
  std::string robot_name = doc.child("robot").attribute("name").as_string();
  RCLCPP_INFO_STREAM(get_logger(), "parsed URDF for " << robot_name);
  makeDirectory(urdf_output_directory_ + "/" + robot_name);
  std::string urdf_path = urdf_output_directory_ + "/" + robot_name + "/output.urdf";
  for (auto & element : doc.child("robot")) {
    std::string element_name = element.name();
    if (element_name == "link") {
      if (
        element.child("visual") && element.child("visual").child("geometry") &&
        element.child("visual").child("geometry").child("mesh")) {
        std::string filename =
          element.child("visual").child("geometry").child("mesh").attribute("filename").as_string();
        filename = resolvePath(filename);
        RCLCPP_INFO_STREAM(get_logger(), "mesh filepath was resolved, filename = " << filename);
        std::string replace_uri = "file://" + filename;
        element.child("visual")
          .child("geometry")
          .child("mesh")
          .attribute("filename")
          .set_value(replace_uri.c_str());
      }
      if (
        element.child("collision") && element.child("collision").child("geometry") &&
        element.child("collision").child("geometry").child("mesh")) {
        std::string filename =
          element.child("collision").child("geometry").child("mesh").attribute("filename").as_string();
        filename = resolvePath(filename);
        RCLCPP_INFO_STREAM(get_logger(), "mesh filepath was resolved, filename = " << filename);
        std::string replace_uri = "file://" + filename;
        element.child("collision")
          .child("geometry")
          .child("mesh")
          .attribute("filename")
          .set_value(replace_uri.c_str());
      }
    }
  }
  doc.save_file(urdf_path.c_str(), "  ");
  RCLCPP_INFO_STREAM(get_logger(), "URDF file was generated at " << urdf_path);
}

}  // namespace unisim_ros2_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(unisim_ros2_control::UniSimRos2ControlComponent)