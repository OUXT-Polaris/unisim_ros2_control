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

#ifndef UNISIM_ROS2_CONTROL__UNISIM_ROS2_CONTROL_HPP_
#define UNISIM_ROS2_CONTROL__UNISIM_ROS2_CONTROL_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_EXPORT __attribute__((dllexport))
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_EXPORT __declspec(dllexport)
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_BUILDING_DLL
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC \
  UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_EXPORT
#else
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC \
  UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_IMPORT
#endif
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC_TYPE \
  UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_LOCAL
#else
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_LOCAL
#endif
#define UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#include <urdf_parser/urdf_parser.h>

#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/asio/ssl/stream_base.hpp>
#include <unisim_client/unisim_openapi_interface/ApiClient.h>

namespace unisim_ros2_control
{
class RobotDescriptionQos : public rclcpp::QoS
{
public:
  explicit RobotDescriptionQos(std::size_t depth = 1) : rclcpp::QoS(depth) { transient_local(); }
};

class UniSimRos2ControlComponent : public rclcpp::Node
{
public:
  UNISIM_ROS2_CONTROL_UNISIM_ROS2_CONTROL_COMPONENT_PUBLIC
  explicit UniSimRos2ControlComponent(const rclcpp::NodeOptions & options);

private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr description);
  std::string resolvePath(std::string filename);
  void makeDirectory(const std::string & path = "/tmp/unisim_ros2_control");
  void copyModelFile(const std::string & from, const std::string & to);
  std::string getExtension(const std::string & path);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  std::string urdf_output_directory_;
};
}  // namespace unisim_ros2_control

#endif  // UNISIM_ROS2_CONTROL__UNISIM_ROS2_CONTROL_HPP_
