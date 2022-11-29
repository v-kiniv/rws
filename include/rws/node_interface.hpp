#ifndef RWS__NODE_INTERFACE_HPP_
#define RWS__NODE_INTERFACE_HPP_

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rws/generic_client.hpp"

namespace rws
{

template <class PublisherClass = rclcpp::GenericPublisher>
class NodeInterface
{
public:
  // virtual NodeInterface();
  // virtual ~NodeInterface();
  /// Create and return a GenericSubscription.
  /**
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param topics_interface NodeTopicsInterface pointer used in parts of the setup.
   * \param topic_name Topic name
   * \param topic_type Topic type
   * \param qos %QoS settings
   * \param callback Callback for new messages of serialized form
   * \param options %Publisher options.
   * Not all publisher options are currently respected, the only relevant options for this
   * publisher are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
   */
  virtual std::shared_ptr<rclcpp::GenericSubscription> create_generic_subscription(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)> callback,
    const rclcpp::SubscriptionOptions & options = (rclcpp::SubscriptionOptions())) = 0;

  /// Create and return a GenericPublisher.
  /**
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param topics_interface NodeTopicsInterface pointer used in parts of the setup
   * \param topic_name Topic name
   * \param topic_type Topic type
   * \param qos %QoS settings
   * \param options %Publisher options.
   * Not all publisher options are currently respected, the only relevant options for this
   * publisher are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
   */
  virtual std::shared_ptr<PublisherClass> create_generic_publisher(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
    const rclcpp::PublisherOptions & options = (rclcpp::PublisherOptions())) = 0;

  /// Create a generic service client with a given type.
  virtual GenericClient::SharedPtr create_generic_client(
    const std::string & service_name, const std::string & service_type,
    const rmw_qos_profile_t & qos_profile, rclcpp::CallbackGroup::SharedPtr group) = 0;

  virtual rclcpp::Time now() const = 0;
  virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;
  virtual rclcpp::node_interfaces::NodeGraphInterface::SharedPtr get_node_graph_interface() = 0;
  virtual rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
  get_node_services_interface() = 0;
  virtual rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() = 0;
  virtual std::map<std::string, std::vector<std::string>> get_service_names_and_types() const = 0;
  virtual std::map<std::string, std::vector<std::string>> get_service_names_and_types_by_node(
    const std::string & node_name, const std::string & namespace_) const = 0;
  virtual std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const = 0;
  virtual std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_info_by_topic(
    const std::string & topic_name, bool no_mangle = false) const = 0;
  virtual std::vector<rclcpp::TopicEndpointInfo> get_publishers_info_by_topic(
    const std::string & topic_name, bool no_mangle = false) const = 0;
  virtual std::vector<std::string> get_node_names() const = 0;
};

}  // namespace rws

#endif  // RWS__NODE_INTERFACE_HPP_
