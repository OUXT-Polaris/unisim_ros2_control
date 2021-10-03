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

#include <unisim_ros2_control/unisim_client.hpp>

namespace unisim_ros2_control
{
UnisimClient::UnisimClient(const std::string & address, std::int16_t port)
{
  configure(address, port);
}

void UnisimClient::configure(const std::string & address, std::int16_t port)
{
  api_configuration_ = std::make_shared<io::swagger::client::api::ApiConfiguration>();
  api_client_ = std::make_shared<io::swagger::client::api::ApiClient>();
  std::string base_url = "http://" + address + ":" + std::to_string(port) + "/v2/";
  api_configuration_->setBaseUrl(base_url);
  web::http::client::http_client_config config;
  api_configuration_->setHttpConfig(config);
  api_client_->setConfiguration(api_configuration_);
  api_ptr_ = std::make_shared<io::swagger::client::api::DefaultApi>(api_client_);
}
}  // namespace unisim_ros2_control