#ifndef RWS__NODE_INTERFACE_IMPL_HPP_
#define RWS__NODE_INTERFACE_IMPL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rws/node_interface.hpp"
#include "rws/typesupport_helpers.hpp"

namespace rws
{

class NodeInterfaceImpl : public rws::NodeInterface<>
{
public:
  NodeInterfaceImpl(std::shared_ptr<rclcpp::Node> node) : node_(node) {}
  ~NodeInterfaceImpl() {}

  std::shared_ptr<rclcpp::GenericSubscription> create_generic_subscription(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)> callback,
    const rclcpp::SubscriptionOptions & options = (rclcpp::SubscriptionOptions()))
  {
    auto ts_lib = rws::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");

    auto subscription = std::make_shared<rclcpp::GenericSubscription>(
      node_->get_node_topics_interface()->get_node_base_interface(), std::move(ts_lib), topic_name,
      topic_type, qos, callback, options);

    node_->get_node_topics_interface()->add_subscription(subscription, options.callback_group);

    return subscription;
  }

  std::shared_ptr<rclcpp::GenericPublisher> create_generic_publisher(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
    const rclcpp::PublisherOptions & options = (rclcpp::PublisherOptions()))
  {
    auto ts_lib = rws::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
    auto pub = std::make_shared<rclcpp::GenericPublisher>(
      node_->get_node_topics_interface()->get_node_base_interface(), std::move(ts_lib), topic_name,
      topic_type, qos, options);
    node_->get_node_topics_interface()->add_publisher(pub, options.callback_group);
    return pub;
  }

  GenericClient::SharedPtr create_generic_client(
    const std::string & service_name, const std::string & service_type,
    const rmw_qos_profile_t & qos_profile, rclcpp::CallbackGroup::SharedPtr group)
  {
    rcl_client_options_t options = rcl_client_get_default_options();
    options.qos = qos_profile;

    auto cli = GenericClient::make_shared(
      node_->get_node_base_interface().get(), node_->get_node_graph_interface(), service_name,
      service_type, options);

    auto cli_base_ptr = std::dynamic_pointer_cast<rclcpp::ClientBase>(cli);
    node_->get_node_services_interface()->add_client(cli_base_ptr, group);
    return cli;
  }

  rclcpp::Time now() const { return node_->now(); }
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr get_node_graph_interface()
  {
    return node_->get_node_graph_interface();
  }
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr get_node_services_interface()
  {
    return node_->get_node_services_interface();
  }
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface()
  {
    return node_->get_node_topics_interface();
  }
  std::map<std::string, std::vector<std::string>> get_service_names_and_types() const
  {
    return node_->get_service_names_and_types();
  }
  std::map<std::string, std::vector<std::string>> get_service_names_and_types_by_node(
    const std::string & node_name, const std::string & namespace_) const
  {
    return node_->get_service_names_and_types_by_node(node_name, namespace_);
  }
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const
  {
    return node_->get_topic_names_and_types();
  }
  std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_info_by_topic(
    const std::string & topic_name, bool no_mangle = false) const
  {
    return node_->get_subscriptions_info_by_topic(topic_name, no_mangle);
  }
  std::vector<rclcpp::TopicEndpointInfo> get_publishers_info_by_topic(
    const std::string & topic_name, bool no_mangle = false) const
  {
    return node_->get_publishers_info_by_topic(topic_name, no_mangle);
  }
  std::vector<std::string> get_node_names() const { return node_->get_node_names(); }

private:
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace rws

#endif  // RWS__NODE_INTERFACE_IMPL_HPP_