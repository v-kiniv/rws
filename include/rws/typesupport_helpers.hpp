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

#ifndef RWS__TYPESUPPORT_HELPERS_HPP_
#define RWS__TYPESUPPORT_HELPERS_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <tuple>

#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/service_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rws
{

using rosidl_typesupport_introspection_cpp::MessageMembers;

const size_t MSG_MEM_BLOCK_SIZE = 1024;
extern const char * ts_identifier;

/// Load the type support library for the given type.
/// This is a thread-safe version of rclcpp::get_typesupport_library.
/// Note: this function will cache the loaded libraries till the end of the program.
/**
 * \param[in] type The topic or service type, e.g. "std_msgs/msg/String" or "std_srvs/srv/Trigger"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \return A shared library
 */
std::shared_ptr<rcpputils::SharedLibrary> get_typesupport_library(
  const std::string & type, const std::string & typesupport_identifier);

/// Extract the service type support handle from the library.
/**
 * The library needs to match the service type. The shared library must stay loaded for the lifetime of the result.
 * \param[in] type The service type, e.g. "std_srvs/srv/Trigger"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_introspection_cpp"
 * \param[in] library The shared type support library
 * \return A type support handle for the service
 */
const rosidl_service_type_support_t * get_service_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

/// Get type string from a serialized message members.
/**
 * \param[in] members The message members
 * \return The type string, e.g. "std_msgs/msg/String"
 */
std::string get_type_from_message_members(const MessageMembers * members);

/// Split namespace and node name into a tuple.
/**
 * \param[in] type The type string, e.g. "some_ns/some_node"
 * \return A tuple of namespace and node name
 */
std::tuple<std::string, std::string> split_ns_node_name(const std::string & node_name);

/// Convert ros1 message type string to ros2 style type string.
/**
 * \param[in] type The ros1 message type string, e.g. "std_msgs/String"
 * \return The ros2 style type string, e.g. "std_msgs/msg/String"
 */
std::string message_type_to_ros2_style(const std::string & ros1_msg_type);

/// Convert ros2 style type string to ros1 style type string.
/**
 * \param[in] type The ros2 style type string, e.g. "std_msgs/msg/String"
 * \return The ros1 message type string, e.g. "std_msgs/String"
 */
std::string message_type_to_ros1_style(const std::string & ros2_msg_type);

/// Allocate memory for a ros message based on serialized message members.
/// It will also prealocate std::string objects in the allocated memory chunk,
/// so RMW deserialization will not cause segfault on Linux systems.
/**
 * \param[in] members The message members
 * \return A shared pointer to the allocated memory
 */
std::shared_ptr<void> allocate_message(const MessageMembers * members);

}  // namespace rws

#endif  // RWS__TYPESUPPORT_HELPERS_HPP_