// Copyright 2022 Vasily Kiniv
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

#ifndef RWS__NODE_HPP_
#define RWS__NODE_HPP_

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rws/connector.hpp"
#include "rws/generic_client.hpp"

namespace rws
{

using json = nlohmann::json;
class ClientHandler
{
public:
  ClientHandler(
    int client_id, std::shared_ptr<rws::NodeInterface<>> node,
    std::shared_ptr<Connector<>> connector, bool rosbridge_compatible,
    std::function<void(std::string & msg)> callback,
    std::function<void(std::vector<std::uint8_t> & msg)> binary_callback);
  json process_message(json & msg);

  ~ClientHandler();

private:
  int client_id_;
  std::shared_ptr<rws::NodeInterface<>> node_;
  std::shared_ptr<rws::Connector<>> connector_;
  bool rosbridge_compatible_;
  std::function<void(std::string & msg)> callback_;
  std::function<void(std::vector<std::uint8_t> & msg)> binary_callback_;

  std::map<std::string, std::function<void()>> subscriptions_;
  std::map<std::string, std::function<void()>> publishers_;
  std::map<std::string, std::string> publisher_type_;
  std::map<std::string, std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)>>
    publisher_cb_;
  std::map<std::string, std::shared_ptr<rws::GenericClient>> clients_;

  rclcpp::Logger get_logger()
  {
    return rclcpp::get_logger(std::string("client_handler_") + std::to_string(client_id_));
  }

  void send_message(std::string & msg);
  void send_message(std::vector<std::uint8_t> & msg);

  // Topic handlers
  bool subscribe_to_topic(const json & request, json & response_out);
  bool unsubscribe_from_topic(const json & request, json & response_out);
  bool advertise_topic(const json & request, json & response_out);
  bool unadvertise_topic(const json & request, json & response_out);
  bool publish_to_topic(const json & request, json & response_out);
  void subscription_callback(topic_params params, std::shared_ptr<const rclcpp::SerializedMessage> message);

  // Service handlers
  bool call_service(const json & request, json & response_out);
  bool call_external_service(const json & request, json & response_out);
};

}  // namespace rws

#endif  // RWS__NODE_HPP_