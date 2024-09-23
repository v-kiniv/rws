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

#include "rws/translate.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rws/typesupport_helpers.hpp"
#include "serdes.hpp"

namespace rws
{

static rclcpp::Logger get_logger() { return rclcpp::get_logger("rws::translate"); }

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ServiceMembers;

template <typename T>
static void deserialize_field(cycdeser & deser, const MessageMember * member, json & field)
{
  T val;
  if (!member->is_array_) {
    deser >> val;
    field = val;
  } else if (member->array_size_ && !member->is_upper_bound_) {
    field = json::array();
    for (size_t i = 0; i < member->array_size_; i++) {
      deser >> val;
      field[i] = val;
    }
  } else {
    field = json::array();
    uint32_t seq_size;
    deser >> seq_size;

    for (size_t i = 0; i < seq_size; i++) {
      deser >> val;
      field[i] = val;
    }
  }
}

static void serialized_message_to_json(cycdeser & deser, const MessageMembers * members, json & j)
{
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto * member = members->members_ + i;

    switch (member->type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        deserialize_field<bool>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        deserialize_field<uint8_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        deserialize_field<char>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
        deserialize_field<float>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        deserialize_field<double>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        deserialize_field<int8_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        deserialize_field<int16_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        deserialize_field<uint16_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        deserialize_field<int32_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        deserialize_field<uint32_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        deserialize_field<int64_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        deserialize_field<uint64_t>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        deserialize_field<std::string>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        deserialize_field<std::wstring>(deser, member, j[member->name_]);
        break;

      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        auto sub_members = (const MessageMembers *)member->members_->data;
        if (!member->is_array_) {
          serialized_message_to_json(deser, sub_members, j[member->name_]);
        } else {
          size_t array_size = 0;

          if (member->array_size_ && !member->is_upper_bound_) {
            array_size = member->array_size_;
          } else {
            array_size = deser.deserialize_len(1);
          }

          if (array_size != 0 && !member->get_function) {
            throw std::runtime_error("unexpected error: get_function function is null");
          }
          for (size_t index = 0; index < array_size; ++index) {
            serialized_message_to_json(deser, sub_members, j[member->name_][index]);
          }
        }
        break;
      }
      default:
        throw std::runtime_error("unknown type");
    }
  }
}

json serialized_message_to_json(const std::string & msg_type, ConstSharedMessage msg)
{
  auto library = rws::get_typesupport_library(msg_type, rws::ts_identifier);
  auto ts = rclcpp::get_typesupport_handle(msg_type, rws::ts_identifier, *library);
  auto members = static_cast<const MessageMembers *>(ts->data);
  auto rcl_msg = &msg->get_rcl_serialized_message();

  cycdeser deser(rcl_msg->buffer, rcl_msg->buffer_length);
  json j;
  serialized_message_to_json(deser, members, j);

  return j;
}

template <typename T>
static void serialize_field(
  const MessageMember * member, json & field, cycser & ser, T default_value)
{
  if (!member->is_array_) {
    ser << (field.is_null() ? default_value : field.get<T>());
  } else if (member->array_size_ && !member->is_upper_bound_) {
    for (size_t i = 0; i < member->array_size_; i++) {
      ser << (field.is_null() || field[i].is_null() ? default_value : field[i].get<T>());
    }
  } else {
    uint32_t seq_size = field.size();
    ser << seq_size;

    for (size_t i = 0; i < seq_size; i++) {
      ser << (field.is_null() || field[i].is_null() ? default_value : field[i].get<T>());
    }
  }
}

static void json_to_serialized_message(cycser & ser, const MessageMembers * members, const json & j)
{
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto member = members->members_ + i;

    if (strcmp(member->name_, "structure_needs_at_least_one_member") == 0) {
      continue;
    }

    auto found_field = j.find(member->name_);

    json field;
    if (found_field == j.end()) {
      RCLCPP_INFO(
        get_logger(), "Field '%s' is not in json, default: %p", member->name_,
        member->default_value_);
    } else {
      field = *found_field;
    }

    switch (member->type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        serialize_field<bool>(member, field, ser, false);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        serialize_field<uint8_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        serialize_field<char>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
        serialize_field<float>(member, field, ser, 0.0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        serialize_field<double>(member, field, ser, 0.0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        serialize_field<int8_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        serialize_field<int16_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        serialize_field<uint16_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        serialize_field<int32_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        serialize_field<uint32_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        serialize_field<int64_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        serialize_field<uint64_t>(member, field, ser, 0);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        serialize_field<std::string>(member, field, ser, std::string(""));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        serialize_field<std::wstring>(member, field, ser, std::wstring(L""));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        auto sub_members = static_cast<const MessageMembers *>(member->members_->data);
        if (!member->is_array_) {
          json_to_serialized_message(ser, sub_members, field);
        } else {
          size_t array_size = 0;

          if (member->array_size_ && !member->is_upper_bound_) {
            array_size = member->array_size_;
          } else {
            if (field.is_array()) {
              array_size = field.size();
            }

            // Serialize length
            ser << (uint32_t)array_size;
          }

          for (size_t index = 0; index < array_size; ++index) {
            json_to_serialized_message(ser, sub_members, field[member->name_][index]);
          }
        }
        break;
      }
      default:
        throw std::runtime_error("unknown type");
    }
  }
}

SharedMessage json_to_serialized_message(const std::string & msg_type, const json & j)
{
  auto library = rws::get_typesupport_library(msg_type, rws::ts_identifier);
  auto ts = rclcpp::get_typesupport_handle(msg_type, rws::ts_identifier, *library);
  auto members = static_cast<const MessageMembers *>(ts->data);

  auto msg = std::make_shared<rclcpp::SerializedMessage>(0);
  auto rcl_msg = &msg->get_rcl_serialized_message();
  std::vector<unsigned char> buffer;
  cycser ser(buffer);

  json_to_serialized_message(ser, members, j);

  msg->reserve(buffer.size());
  memcpy(rcl_msg->buffer, buffer.data(), buffer.size());
  rcl_msg->buffer_length = buffer.size();

  return msg;
}

SharedMessage json_to_serialized_service_request(const std::string & srv_type, const json & j)
{
  auto library = rws::get_typesupport_library(srv_type, rws::ts_identifier);
  auto ts = rws::get_service_typesupport_handle(srv_type, rws::ts_identifier, *library);
  auto srv_members = static_cast<const ServiceMembers *>(ts->data);
  auto request_members = srv_members->request_members_;

  auto msg = std::make_shared<rclcpp::SerializedMessage>(0);
  auto rcl_msg = &msg->get_rcl_serialized_message();
  std::vector<unsigned char> buffer;
  cycser ser(buffer);

  json_to_serialized_message(ser, request_members, j);

  msg->reserve(buffer.size());
  memcpy(rcl_msg->buffer, buffer.data(), buffer.size());
  rcl_msg->buffer_length = buffer.size();

  return msg;
}

json serialized_service_response_to_json(const std::string & srv_type, ConstSharedMessage msg)
{
  auto library = rws::get_typesupport_library(srv_type, rws::ts_identifier);
  auto ts = rws::get_service_typesupport_handle(srv_type, rws::ts_identifier, *library);
  auto srv_members = static_cast<const ServiceMembers *>(ts->data);
  auto response_members = srv_members->response_members_;

  auto rcl_msg = &msg->get_rcl_serialized_message();

  cycdeser deser(rcl_msg->buffer, rcl_msg->buffer_length);
  json j;
  serialized_message_to_json(deser, response_members, j);

  return j;
}

static std::string members_to_meta(
  const MessageMembers * members, std::map<std::string, std::string> & deps,
  bool rosbridge_compatible = false, std::string parent_name = "")
{
  std::stringstream s;

  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto member = members->members_ + i;
    std::string name = member->name_;
    if (name == "structure_needs_at_least_one_member") {
      continue;
    }
    if (rosbridge_compatible && name == "nanosec" && parent_name == "stamp") {
      name = "nsec";
    }

    std::string b =
      member->is_array_
        ? !member->array_size_ ? "[]" : "[" + std::to_string(member->array_size_) + "]"
        : "";

    switch (member->type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        s << "bool" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:  // byte is legacy type
        // s << "byte" << b << " " << name << "\n";
        // break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:  // char is legacy type
        // s << "char" << b << " " << name << "\n";
        // break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        s << "uint8" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        s << "int8" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
        s << "float32" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        s << "float64" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        s << "int16" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        s << "uint16" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        s << "int32" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        s << "uint32" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        s << "int64" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        s << "uint64" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        s << "string" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        s << "wstring" << b << " " << name << "\n";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        auto sub_members = static_cast<const MessageMembers *>(member->members_->data);
        auto msg_path = get_type_from_message_members(sub_members);

        s << msg_path << b << " " << name << "\n";

        if (deps.count(msg_path) == 0) {
          deps[msg_path] = members_to_meta(sub_members, deps, rosbridge_compatible, name);
        }
      } break;
      default:
        throw std::runtime_error("unknown type");
    }
  }

  return s.str();
}

std::string generate_message_meta(const std::string & msg_type, bool rosbridge_compatible)
{
  auto library = rws::get_typesupport_library(msg_type, rws::ts_identifier);
  auto ts = rclcpp::get_typesupport_handle(msg_type, rws::ts_identifier, *library);
  auto members = static_cast<const MessageMembers *>(ts->data);

  std::stringstream s;
  std::map<std::string, std::string> deps;
  s << members_to_meta(members, deps, rosbridge_compatible);

  for (auto & dep : deps) {
    s << "============\n";
    s << "MSG: " << dep.first << "\n";
    s << dep.second;
  }

  return s.str();
}

}  // namespace rws