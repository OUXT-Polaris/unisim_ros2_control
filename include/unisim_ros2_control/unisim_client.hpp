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

#include <ApiClient.h>
#include <api/DefaultApi.h>

#include <string>

namespace unisim_ros2_control
{
class UnisimClient
{
public:
  UnisimClient(const std::string & address = "localhost", std::int16_t port = 8080);
  template <typename T>
  UnisimClient(std::string address = "localhost", T port = 8080)
  {
    if (port > std::numeric_limits<std::uint16_t>::max()) {
      throw std::out_of_range(
        "port number is," + std::to_string(port) +
        "overs range of std::uint16_t, maximun value of port should be " +
        std::to_string(std::numeric_limits<std::uint16_t>::max()));
    }
    if (port < std::numeric_limits<std::uint16_t>::min()) {
      throw std::out_of_range(
        "port number is," + std::to_string(port) +
        "overs range of std::uint16_t, minimum value of port should be " +
        std::to_string(std::numeric_limits<std::uint16_t>::min()));
    }
    configure(address, static_cast<std::uint16_t>(port));
  }

private:
  void configure(const std::string & address, std::int16_t port);
  std::shared_ptr<io::swagger::client::api::ApiClient> api_client_;
  std::shared_ptr<io::swagger::client::api::ApiConfiguration> api_configuration_;
  std::shared_ptr<io::swagger::client::api::DefaultApi> api_ptr_;
  std::uint16_t api_port_;
};
}  // namespace unisim_ros2_control