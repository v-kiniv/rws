
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "generic_publisher_mock.hpp"
#include "rws/node_interface.hpp"

class NodeMock : public rws::NodeInterface<GenericPublisherMock>
{
public:
  // using rws::NodeInterface<GenericPublisherMock>::NodeInterface;
  MOCK_METHOD(
    std::shared_ptr<rclcpp::GenericSubscription>, create_generic_subscription,
    (const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
     std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)> callback,
     const rclcpp::SubscriptionOptions & options),
    ());

  MOCK_METHOD(
    std::shared_ptr<GenericPublisherMock>, create_generic_publisher,
    (const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
     const rclcpp::PublisherOptions & options),
    ());

  MOCK_METHOD(
    rws::GenericClient::SharedPtr, create_generic_client,
    (const std::string & service_name, const std::string & service_type,
     const rmw_qos_profile_t & qos_profile, rclcpp::CallbackGroup::SharedPtr group),
    ());

  MOCK_METHOD(rclcpp::Time, now, (), (const));
  MOCK_METHOD(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr, get_node_base_interface, (), ());
  MOCK_METHOD(
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr, get_node_graph_interface, (), ());
  MOCK_METHOD(
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr, get_node_services_interface, (), ());
  MOCK_METHOD(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr, get_node_topics_interface, (), ());
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>), get_service_names_and_types, (), (const));
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>), get_service_names_and_types_by_node,
    (const std::string & node_name, const std::string & namespace_), (const));
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>), get_topic_names_and_types, (), (const));
  MOCK_METHOD(
    std::vector<rclcpp::TopicEndpointInfo>, get_subscriptions_info_by_topic,
    (const std::string & topic_name, bool no_mangle), (const));
  MOCK_METHOD(
    std::vector<rclcpp::TopicEndpointInfo>, get_publishers_info_by_topic,
    (const std::string & topic_name, bool no_mangle), (const));
  MOCK_METHOD(std::vector<std::string>, get_node_names, (), (const));
};