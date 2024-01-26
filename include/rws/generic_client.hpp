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

#ifndef RWS__GENERIC_CLIENT_HPP_
#define RWS__GENERIC_CLIENT_HPP_

#include "rclcpp/client.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rmw/rmw.h"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rws/typesupport_helpers.hpp"

namespace rws
{
using rosidl_typesupport_introspection_cpp::ServiceMembers;

class GenericClient : public rclcpp::ClientBase
{
public:
  using SharedRequest = std::shared_ptr<rclcpp::SerializedMessage>;
  using SharedResponse = std::shared_ptr<rclcpp::SerializedMessage>;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;

  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;

  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;

  using CallbackType = std::function<void(SharedFuture)>;
  using CallbackWithRequestType = std::function<void(SharedFutureWithRequest)>;

  RCLCPP_SMART_PTR_DEFINITIONS(GenericClient)

  GenericClient(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, std::string service_name,
    std::string service_type, rcl_client_options_t & client_options)
  : rclcpp::ClientBase(node_base, node_graph)
  {
    srv_ts_lib_ = rclcpp::get_typesupport_library(service_type, rws::ts_identifier);
    srv_ts_hdl_ =
      rws::get_service_typesupport_handle(service_type, rws::ts_identifier, *srv_ts_lib_);
    auto srv_members = static_cast<const ServiceMembers *>(srv_ts_hdl_->data);

    auto request_members = srv_members->request_members_;
    auto request_type = get_type_from_message_members(request_members);
    req_ts_lib_ = rclcpp::get_typesupport_library(request_type, rws::ts_identifier);
    req_ts_hdl_ = rclcpp::get_typesupport_handle(request_type, rws::ts_identifier, *req_ts_lib_);

    auto response_members = srv_members->response_members_;
    auto response_type = get_type_from_message_members(response_members);
    res_ts_lib_ = rclcpp::get_typesupport_library(response_type, rws::ts_identifier);
    res_ts_hdl_ = rclcpp::get_typesupport_handle(response_type, rws::ts_identifier, *res_ts_lib_);

    rcl_ret_t ret = rcl_client_init(
      this->get_client_handle().get(), this->get_rcl_node_handle(), srv_ts_hdl_,
      service_name.c_str(), &client_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = this->get_rcl_node_handle();
        // this will throw on any validation problem
        rcl_reset_error();
        rclcpp::expand_topic_or_service_name(
          service_name, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle),
          true);
      }
      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
    }
  }

  virtual ~GenericClient() {}

  std::shared_ptr<void> create_response() override
  {
    auto srv_members = static_cast<const ServiceMembers *>(srv_ts_hdl_->data);
    return allocate_message(srv_members->response_members_);
  }

  std::shared_ptr<rmw_request_id_t> create_request_header() override
  {
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) override
  {
    std::unique_lock<std::mutex> lock(pending_requests_mutex_);
    int64_t sequence_number = request_header->sequence_number;

    auto ser_response = std::make_shared<rclcpp::SerializedMessage>();
    rmw_ret_t r =
      rmw_serialize(response.get(), res_ts_hdl_, &ser_response->get_rcl_serialized_message());
    if (r != RMW_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED("rws", "Failed to serialize service response. Ignoring...");
      return;
    }

    // TODO(esteve) this should throw instead since it is not expected to happen in the first place
    if (this->pending_requests_.count(sequence_number) == 0) {
      RCUTILS_LOG_ERROR_NAMED("rws", "Received invalid sequence number. Ignoring...");
      return;
    }
    auto tuple = this->pending_requests_[sequence_number];
    auto call_promise = std::get<0>(tuple);
    auto callback = std::get<1>(tuple);
    auto future = std::get<2>(tuple);
    this->pending_requests_.erase(sequence_number);
    // Unlock here to allow the service to be called recursively from one of its callbacks.
    lock.unlock();

    call_promise->set_value(ser_response);
    callback(future);
  }

  SharedFuture async_send_request(SharedRequest request)
  {
    return async_send_request(request, [](SharedFuture) {});
  }

  SharedFuture async_send_request(SharedRequest request, CallbackType && cb)
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    int64_t sequence_number;

    auto srv_members = static_cast<const ServiceMembers *>(srv_ts_hdl_->data);
    auto req_members = srv_members->request_members_;
    auto buf = allocate_message(req_members);

    if(req_members->member_count_ > 1 ||
       std::strcmp(req_members->members_[0].name_, "structure_needs_at_least_one_member") != 0) {
      const rmw_serialized_message_t * sm = &request->get_rcl_serialized_message();
      rmw_ret_t rmw_ret = rmw_deserialize(sm, req_ts_hdl_, buf.get());
      if(RMW_RET_OK != rmw_ret) {
        rclcpp::exceptions::throw_from_rcl_error(rmw_ret, "failed to deserialize request");
      }
    }

    rcl_ret_t ret = rcl_send_request(get_client_handle().get(), buf.get(), &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }

    SharedPromise call_promise = std::make_shared<Promise>();
    SharedFuture f(call_promise->get_future());
    pending_requests_[sequence_number] =
      std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);
    return f;
  }

private:
  RCLCPP_DISABLE_COPY(GenericClient)

  std::map<int64_t, std::tuple<SharedPromise, CallbackType, SharedFuture>> pending_requests_;
  std::mutex pending_requests_mutex_;
  std::shared_ptr<rcpputils::SharedLibrary> srv_ts_lib_;
  const rosidl_service_type_support_t * srv_ts_hdl_;
  std::shared_ptr<rcpputils::SharedLibrary> req_ts_lib_;
  const rosidl_message_type_support_t * req_ts_hdl_;
  std::shared_ptr<rcpputils::SharedLibrary> res_ts_lib_;
  const rosidl_message_type_support_t * res_ts_hdl_;
};

}  // namespace rws

#endif  // RWS__GENERIC_CLIENT_HPP_