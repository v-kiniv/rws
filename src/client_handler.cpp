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

#include "rws/client_handler.hpp"

#include <cstdio>
#include <nlohmann/json.hpp>

#include "rclcpp/logger.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rws/translate.hpp"

namespace rws
{

using json = nlohmann::json;
using namespace std::chrono_literals;
using std::placeholders::_1;

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

ClientHandler::ClientHandler(
  int client_id, std::shared_ptr<rws::NodeInterface<>> node, std::shared_ptr<Connector<>> connector,
  bool rosbridge_compatible, std::function<void(std::string & msg)> callback,
  std::function<void(std::vector<std::uint8_t> & msg)> binary_callback)
: client_id_(client_id),
  node_(node),
  connector_(connector),
  rosbridge_compatible_(rosbridge_compatible),
  callback_(callback),
  binary_callback_(binary_callback)
{
  RCLCPP_INFO(
    get_logger(), "Constructing client %s(%s)", std::to_string(client_id_).c_str(),
    string_thread_id().c_str());
}

ClientHandler::~ClientHandler()
{
  RCLCPP_INFO(
    get_logger(), "Destroying client %s(%s)", std::to_string(client_id_).c_str(),
    string_thread_id().c_str());
  for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
    it->second();
  }
  for (auto it = publishers_.begin(); it != publishers_.end(); ++it) {
    it->second();
  }
}

json ClientHandler::process_message(json & msg)
{
  bool handled = false;
  json response = {{"id", msg["id"]}, {"result", false}};

  if (!msg.contains("op")) {
    response["error"] = "No op specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return response;
  }

  std::string op = msg["op"];

  if (op == "call_service") {
    handled = call_service(msg, response);
  }

  if (op == "subscribe") {
    handled = subscribe_to_topic(msg, response);
  }

  if (op == "advertise") {
    handled = advertise_topic(msg, response);
  }

  if (op == "unadvertise") {
    handled = unadvertise_topic(msg, response);
  }

  if (op == "publish") {
    handled = publish_to_topic(msg, response);
  }

  if (op == "unsubscribe") {
    handled = unsubscribe_from_topic(msg, response);
  }

  if (!handled) {
    RCLCPP_WARN(get_logger(), "Unhadled request: %s", msg.dump().c_str());
  }

  return response;
}

void ClientHandler::send_message(std::string & msg)
{
  if (this->callback_) {
    this->callback_(msg);
  }
}

void ClientHandler::send_message(std::vector<std::uint8_t> & msg)
{
  if (this->binary_callback_) {
    this->binary_callback_(msg);
  }
}

bool ClientHandler::subscribe_to_topic(const json & msg, json & response)
{
  response["op"] = "subscribe_response";
  if (!msg.contains("topic") || !msg["topic"].is_string()) {
    response["result"] = false;
    response["error"] = "No topic specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  std::string topic = msg["topic"];
  std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();
  if (topics.find(topic) == topics.end()) {
    response["error"] = "Topic " + topic + " not found";
    response["result"] = false;
    RCLCPP_ERROR(
      get_logger(), "Failed to subscribe to topic: %s", response["error"].dump().c_str());
    return true;
  }
  size_t history_depth = 10;
  if (msg.contains("history_depth") && msg["history_depth"].is_number()) {
    history_depth = msg["history_depth"];
  } else if (msg.contains("queue_size") && msg["queue_size"].is_number()) {
    history_depth = msg["queue_size"];
  }
  std::string compression =
    (!msg.contains("compression") || !msg["compression"].is_string()) ? "none" : msg["compression"];

  auto sub_type = topics[topic][0];
  if (subscriptions_.count(topic) == 0) {
    topic_params params(topic, sub_type, history_depth, compression);
    subscriptions_[topic] = connector_->subscribe_to_topic(
      client_id_, params,
      [this, topic, sub_type,
       compression](std::shared_ptr<const rclcpp::SerializedMessage> message) {
        uint32_t secs = node_->now().seconds();
        uint32_t nsecs = node_->now().nanoseconds() - (secs * 1000000000);

        json m = {
          {"op", "publish"},
          {"topic", topic},
        };

        if (compression == "cbor-raw") {
          auto buf = std::vector<std::uint8_t>(
            &message->get_rcl_serialized_message().buffer[0],
            &message->get_rcl_serialized_message()
               .buffer[message->get_rcl_serialized_message().buffer_length]);
          m["msg"] = {{"secs", secs}, {"nsecs", nsecs}, {"bytes", json::binary_t(buf)}};
          std::vector<std::uint8_t> cbor_buf = json::to_cbor(m);
          this->send_message(cbor_buf);
        } else if (compression == "cbor") {
          m["msg"] = rws::serialized_message_to_json(sub_type, std::move(message));
          std::vector<std::uint8_t> buf = json::to_cbor(m);
          this->send_message(buf);
        } else if (compression == "bson") {
          m["msg"] = rws::serialized_message_to_json(sub_type, message);
          std::vector<std::uint8_t> buf = json::to_bson(m);
          this->send_message(buf);
        } else if (compression == "msgpack") {
          m["msg"] = rws::serialized_message_to_json(sub_type, message);
          std::vector<std::uint8_t> buf = json::to_msgpack(m);
          this->send_message(buf);
        } else if (compression == "ubjson") {
          m["msg"] = rws::serialized_message_to_json(sub_type, message);
          std::vector<std::uint8_t> buf = json::to_ubjson(m);
          this->send_message(buf);
        } else if (compression == "bjdata") {
          m["msg"] = rws::serialized_message_to_json(sub_type, message);
          std::vector<std::uint8_t> buf = json::to_bjdata(m);
          this->send_message(buf);
        } else {
          m["msg"] = rws::serialized_message_to_json(sub_type, message);
          std::string json_str = m.dump();
          this->send_message(json_str);
        }
      });

    response["type"] = sub_type;
    response["result"] = true;
  }

  return true;
}

bool ClientHandler::unsubscribe_from_topic(const json & msg, json & response)
{
  response["op"] = "unsubscribe_response";

  std::string topic = msg["topic"];
  if (subscriptions_.count(topic) > 0) {
    subscriptions_[topic]();
    subscriptions_.erase(topic);
    response["result"] = true;
  }

  return true;
}

bool ClientHandler::advertise_topic(const json & msg, json & response)
{
  response["op"] = "advertise_response";
  if (!msg.contains("type") || !msg["type"].is_string()) {
    response["result"] = false;
    response["error"] = "No type specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  if (!msg.contains("topic") || !msg["topic"].is_string()) {
    response["result"] = false;
    response["error"] = "No topic specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  std::string topic = msg["topic"];
  std::string type = rws::message_type_to_ros2_style(msg["type"]);
  size_t history_depth = 10;
  if (msg.contains("history_depth") && msg["history_depth"].is_number()) {
    history_depth = msg["history_depth"];
  } else if (msg.contains("queue_size") && msg["queue_size"].is_number()) {
    history_depth = msg["queue_size"];
  }
  bool latch =
    (msg.contains("latch") && msg["latch"].is_boolean()) ? msg["latch"].get<bool>() : false;
  topic_params params(topic, type, history_depth, latch);

  if (publishers_.count(topic) == 0) {
    publishers_[topic] = connector_->advertise_topic(client_id_, params, publisher_cb_[topic]);
    publisher_type_[topic] = type;
    response["result"] = true;
  }

  return true;
}

bool ClientHandler::unadvertise_topic(const json & msg, json & response)
{
  response["op"] = "unadvertise_response";

  std::string topic = msg["topic"];
  if (publishers_.count(topic) > 0) {
    publishers_[topic]();
    publishers_.erase(topic);
    publisher_cb_.erase(topic);
    publisher_type_.erase(topic);
    response["result"] = true;
  }

  return true;
}

bool ClientHandler::publish_to_topic(const json & msg, json & response)
{
  response["op"] = "publish_response";

  if (!msg.contains("topic")) {
    response["false"] = true;
    response["error"] = "No topic specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  std::string topic = msg["topic"];

  if (publishers_.count(topic) > 0) {
    json msg_json = msg["msg"];
    std::string type = publisher_type_[topic];
    auto serialized_msg = rws::json_to_serialized_message(type, msg_json);
    publisher_cb_[topic](serialized_msg);
    response["result"] = true;
  } else {
    response["result"] = false;
    response["error"] = "Topic was not advertised";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
  }

  return true;
}

bool ClientHandler::call_service(const json & msg, json & response)
{
  if (!msg.contains("service")) {
    RCLCPP_ERROR(get_logger(), "No service specified");
    return true;
  }
  std::string service = msg["service"];

  response["op"] = "service_response";
  response["service"] = service;
  response["result"] = false;

  if (service == "/rosapi/topics_and_raw_types" || service == "/rosapi/topics") {
    response["values"]["topics"] = json::array();
    response["values"]["types"] = json::array();

    std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();
    for (auto it = topics.begin(); it != topics.end(); ++it) {
      response["values"]["topics"].push_back(it->first);
      response["values"]["types"].push_back(it->second[0]);

      if (msg["service"] == "/rosapi/topics_and_raw_types") {
        response["values"]["typedefs_full_text"].push_back(
          rws::generate_message_meta(it->second[0], rosbridge_compatible_));
      }
    }
    response["result"] = true;
    return true;
  }

  if (service == "/rosapi/service_type") {
    std::string service_name = msg["args"]["service"];
    std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();
    if (services.find(service_name) == services.end()) {
      RCLCPP_ERROR(get_logger(), "Service not found: %s", service_name.c_str());
      return true;
    }

    std::string service_type = services[service_name][0];
    response["values"]["type"] = service_type;
    response["result"] = true;
    return true;
  }

  if (service == "/rosapi/nodes") {
    response["values"]["nodes"] = json::array();

    std::vector<std::string> nodes = node_->get_node_names();
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
      response["values"]["nodes"].push_back(*it);
    }

    response["result"] = true;
    return true;
  }

  if (service == "/rosapi/node_details") {
    response["values"]["subscribing"] = json::array();
    response["values"]["publishing"] = json::array();
    response["values"]["services"] = json::array();

    auto [ns, node_name] = split_ns_node_name(msg["args"]["node"]);

    std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();
    for (auto it = topics.begin(); it != topics.end(); ++it) {
      auto subscribers = node_->get_subscriptions_info_by_topic(it->first);
      for (auto sub_it = subscribers.begin(); sub_it != subscribers.end(); ++sub_it) {
        std::string sub_node = sub_it->node_name();
        if (sub_node == node_name) {
          response["values"]["subscribing"].push_back(it->first);
        }
      }

      auto publishers = node_->get_publishers_info_by_topic(it->first);
      for (auto pub_it = publishers.begin(); pub_it != publishers.end(); ++pub_it) {
        std::string pub_node = pub_it->node_name();
        if (pub_node == node_name) {
          response["values"]["publishing"].push_back(it->first);
        }
      }
    }

    try {
      auto services = node_->get_service_names_and_types_by_node(node_name, ns);
      for (auto it = services.begin(); it != services.end(); ++it) {
        response["values"]["services"].push_back(it->first);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Exception while fetching services for node(%s), ns=%s, name=%s: %s",
        msg["args"]["node"].get<std::string>().c_str(), ns.c_str(), node_name.c_str(), e.what());
    }

    response["result"] = true;
    return true;
  }

  return call_external_service(msg, response);
}

bool ClientHandler::call_external_service(const json & msg, json & response)
{
  std::string service_name = msg["service"];
  std::string service_type = msg["type"];

  std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();
  if (services.find(service_name) == services.end()) {
    RCLCPP_ERROR(get_logger(), "Service not found: %s", service_name.c_str());
    return false;
  }

  if (clients_.count(service_type) == 0) {
    clients_[service_type] = node_->create_generic_client(
      service_name, service_type, rmw_qos_profile_services_default, nullptr);
  }

  while (!clients_[service_type]->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      response["result"] = false;
      return false;
    }
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }

  auto serialized_req = json_to_serialized_service_request(service_type, msg["args"]);
  using ServiceResponseFuture = rws::GenericClient::SharedFuture;
  auto response_received_callback = [this, id = msg["id"], service_name,
                                     service_type](ServiceResponseFuture future) {
    json response_json = serialized_service_response_to_json(service_type, future.get());
    json m = {
      {"id", id},
      {"op", "service_response"},
      {"service", service_name},
      {"values", response_json},
      {"result", true},
    };

    std::string json_str = m.dump();
    this->send_message(json_str);
  };
  clients_[service_type]->async_send_request(serialized_req, response_received_callback);

  response["op"] = "call_service";
  response["result"] = true;
  return true;
}

}  // namespace rws