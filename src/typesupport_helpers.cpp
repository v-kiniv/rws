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

#include "rws/typesupport_helpers.hpp"

#include <mutex>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "rcpputils/find_library.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rws
{
const char * ts_identifier = "rosidl_typesupport_introspection_cpp";
namespace
{

std::mutex g_shared_lib_lock_;
std::map<std::string, std::shared_ptr<rcpputils::SharedLibrary>> g_shared_libs_;

// Look for the library in the ament prefix paths.
std::string get_typesupport_library_path(
  const std::string & package_name, const std::string & typesupport_identifier)
{
  const char * dynamic_library_folder;
#ifdef _WIN32
  dynamic_library_folder = "/bin/";
#elif __APPLE__
  dynamic_library_folder = "/lib/";
#else
  dynamic_library_folder = "/lib/";
#endif

  std::string package_prefix;
  try {
    package_prefix = ament_index_cpp::get_package_prefix(package_name);
  } catch (ament_index_cpp::PackageNotFoundError & e) {
    throw std::runtime_error(e.what());
  }

  const std::string library_path = rcpputils::path_for_library(
    package_prefix + dynamic_library_folder, package_name + "__" + typesupport_identifier);
  if (library_path.empty()) {
    throw std::runtime_error(
      "Typesupport library for " + package_name + " does not exist in '" + package_prefix + "'.");
  }
  return library_path;
}

static std::tuple<std::string, std::string, std::string> extract_type_identifier(
  const std::string & full_type)
{
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (
    sep_position_back == std::string::npos || sep_position_back == 0 ||
    sep_position_back == full_type.length() - 1) {
    throw std::runtime_error(
      "Message type is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module =
      full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}

}  // anonymous namespace

std::shared_ptr<rcpputils::SharedLibrary> get_typesupport_library(
  const std::string & type, const std::string & typesupport_identifier)
{
  std::string type_id = type + typesupport_identifier;
  if (g_shared_libs_.count(type_id) == 0) {
    std::lock_guard<std::mutex> guard(g_shared_lib_lock_);
    auto package_name = std::get<0>(extract_type_identifier(type));
    auto library_path = get_typesupport_library_path(package_name, typesupport_identifier);
    g_shared_libs_[type_id] = std::make_shared<rcpputils::SharedLibrary>(library_path);
  }

  return g_shared_libs_[type_id];
}

const rosidl_service_type_support_t * get_service_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library)
{
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(type);

  auto mk_error =
    [&package_name, &type_name](auto reason) {
      std::stringstream rcutils_dynamic_loading_error;
      rcutils_dynamic_loading_error
        << "Something went wrong loading the typesupport library for message type " << package_name
        << "/" << type_name << ". " << reason;
      return rcutils_dynamic_loading_error.str();
    };

  try {
    std::string symbol_name = typesupport_identifier + "__get_service_type_support_handle__" +
                              package_name + "__" +
                              (middle_module.empty() ? "srv" : middle_module) + "__" + type_name;

    const rosidl_service_type_support_t * (*get_ts)() = nullptr;
    // This will throw runtime_error if the symbol was not found.
    get_ts = reinterpret_cast<decltype(get_ts)>(library.get_symbol(symbol_name));
    return get_ts();
  } catch (std::runtime_error &) {
    throw std::runtime_error{mk_error("Library could not be found.")};
  }
}

std::string get_type_from_message_members(const MessageMembers * members)
{
  std::string s = members->message_namespace_;
  s.replace(s.find("::"), sizeof("::") - 1, "/");
  return s + "/" + members->message_name_;
}

std::tuple<std::string, std::string> split_ns_node_name(const std::string & node_name)
{
  auto sep_position = node_name.find_last_of("/");
  if (sep_position == std::string::npos) {
    throw std::runtime_error("Node name is not of the form namespace/node and cannot be processed");
  }

  std::string ns = node_name.substr(0, sep_position);
  std::string node = node_name.substr(sep_position + 1);

  return std::make_tuple(ns, node);
}

std::string message_type_to_ros2_style(const std::string & ros1_msg_type)
{
  std::string s = ros1_msg_type;
  if (s.find("/msg/") != std::string::npos) {
    return ros1_msg_type;
  }
  s.replace(s.find("/"), sizeof("/") - 1, "/msg/");
  return s;
}

std::string message_type_to_ros1_style(const std::string & ros2_msg_type)
{
  std::string s = ros2_msg_type;
  if (s.find("/msg/") != std::string::npos) {
    s.replace(s.find("/msg/"), sizeof("/msg/") - 1, "/");
  }
  return s;
}

std::shared_ptr<void> allocate_message(const MessageMembers * members)
{
  void * buf = new uint8_t[members->size_of_];
  members->init_function(buf, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return std::shared_ptr<void>(
    buf,
    [members](void * p)
    {
      members->fini_function(p);
      delete[] reinterpret_cast<uint8_t *>(p);
    });
}

}  // namespace rws