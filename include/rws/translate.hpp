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

#ifndef RWS__TRANSLATE_HPP_
#define RWS__TRANSLATE_HPP_

#include <nlohmann/json.hpp>
#include <string>

#include "rclcpp/serialized_message.hpp"

namespace rws
{

using json = nlohmann::json;
using SharedMessage = std::shared_ptr<rclcpp::SerializedMessage>;
using ConstSharedMessage = std::shared_ptr<const rclcpp::SerializedMessage>;

/// Translate serialized message to json object
/**
 * \param msg_type Message type, e.g. "std_msgs/String"
 * \param[in] msg Serialized message
 * \return Json representation of the message
 */
json serialized_message_to_json(const std::string & msg_type, ConstSharedMessage msg);

/// Translate json object to serialized message
/**
 * \param[in] msg_type Message type, e.g. "std_msgs/msg/String"
 * \param[in] j Json representation of the message
 * \return Serialized message
 */
SharedMessage json_to_serialized_message(const std::string & msg_type, const json & j);

/// Translate json object to serialized service request
/**
 * \param[in] srv_type Service type, e.g. "std_srvs/srv/SetBool"
 * \param[in] j Json representation of service request
 * \return Serialized service request
 */
SharedMessage json_to_serialized_service_request(const std::string & srv_type, const json & j);

/// Translate serialized service response to json object
/**
 * \param[in] srv_type Service type, e.g. "std_srvs/srv/SetBool"
 * \param[in] msg Serialized service response
 * \return Json representation of the service response
 */
json serialized_service_response_to_json(const std::string & srv_type, ConstSharedMessage msg);

/// Generate textual representation of the message structure
/**
 * \param[in] msg_type Message type, e.g. "std_msgs/msg/String"
 * \param[in] rosbridge_compatible Is Rosbidge compatible, e.g. replace nanosec with nsec
 * \return String representation of the message structure
 */
std::string generate_message_meta(const std::string & msg_type, bool rosbridge_compatible = false);

}  // namespace rws

#endif  // RWS__TRANSLATE_HPP_