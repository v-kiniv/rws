
#include "rws/client_handler.hpp"

#include <gtest/gtest.h>

#include "generic_publisher_mock.hpp"
#include "rws/node_interface_impl.hpp"

namespace rws
{

using json = nlohmann::json;

class ClientHandlerFixture : public testing::Test
{
public:
  ClientHandlerFixture() {}

  void SetUp() override {}

  void TearDown() override {}

protected:
};

TEST_F(ClientHandlerFixture, subsribe_to_topic_is_thread_safe)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<rws::NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<rws::Connector<>>(node_interface);
  auto pub = server_node->create_generic_publisher("/test", "std_msgs/msg/String", rclcpp::QoS(10));
  auto json_o = json::parse(R"(
    {
      "compression": "none",
      "op": "subscribe",
      "topic": "/test"
    }
  )");
  const int nodes_count = 50;

  std::vector<std::thread> threads;
  std::vector<std::shared_ptr<rws::ClientHandler>> nodes;
  for (int ti = 0; ti < nodes_count; ti++) {
    nodes.push_back(std::make_shared<rws::ClientHandler>(
      ti, node_interface, connector, true, [](std::string &) {},
      [](std::vector<std::uint8_t> &) {}));
    threads.push_back(std::thread([ti, nodes, &json_o]() {
      for (int i = 0; i < 1000; i++) {
        nodes[ti]->process_message(json_o);
      }
    }));
  }

  for (int ti = 0; ti < nodes_count; ti++) {
    threads[ti].join();
  }

  EXPECT_NE(server_node, nullptr);
}

TEST_F(ClientHandlerFixture, advertise_topic_is_thread_safe)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<rws::NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<rws::Connector<>>(node_interface);
  auto json_o = json::parse(R"(
    {
      "op": "advertise",
      "history_depth": 10,
      "type": "std_msgs/msg/String",
      "topic": "/test",
      "latch": false
    }
  )");
  const int nodes_count = 50;

  std::vector<std::thread> threads;
  std::vector<std::shared_ptr<rws::ClientHandler>> nodes;
  for (int ti = 0; ti < nodes_count; ti++) {
    nodes.push_back(std::make_shared<rws::ClientHandler>(
      ti, node_interface, connector, false, [](std::string &) {},
      [](std::vector<std::uint8_t> &) {}));
    threads.push_back(std::thread([ti, nodes, &json_o]() {
      for (int i = 0; i < 1000; i++) {
        nodes[ti]->process_message(json_o);
      }
    }));
  }

  for (int ti = 0; ti < nodes_count; ti++) {
    threads[ti].join();
  }

  EXPECT_NE(server_node, nullptr);
}

TEST_F(ClientHandlerFixture, rosapi_topic_and_raw_types_is_thread_safe)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<rws::NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<rws::Connector<>>(node_interface);
  auto json_o = json::parse(R"(
    {
      "op": "call_service",
      "service": "/rosapi/topics_and_raw_types"
    }
  )");
  const int nodes_count = 20;

  std::vector<std::thread> threads;
  std::vector<std::shared_ptr<rws::ClientHandler>> nodes;
  for (int ti = 0; ti < nodes_count; ti++) {
    nodes.push_back(std::make_shared<rws::ClientHandler>(
      ti, node_interface, connector, true, [](std::string &) {},
      [](std::vector<std::uint8_t> &) {}));
    threads.push_back(std::thread([ti, nodes, &json_o]() {
      for (int i = 0; i < 1000; i++) {
        nodes[ti]->process_message(json_o);
      }
    }));
  }

  for (int ti = 0; ti < nodes_count; ti++) {
    threads[ti].join();
  }

  EXPECT_NE(server_node, nullptr);
}

TEST_F(ClientHandlerFixture, subscription_callback_throttles_messages)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<rws::NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<rws::Connector<>>(node_interface);

  std::vector<std::string> messages;
  std::function<void(std::string &)> callback = [&messages](std::string & message) {
    messages.push_back(message);
  };
  std::function<void(std::vector<std::uint8_t> &)> binary_callback = [](std::vector<std::uint8_t> &) {};

  ClientHandler handler(0, node_interface, connector, true, callback, binary_callback);

  topic_params params;
  params.topic = "/test_topic";
  params.type = "std_msgs/msg/String";
  params.throttle_rate = 100;

  auto message = std::make_shared<const rclcpp::SerializedMessage>();

  // Send multiple messages rapidly
  handler.subscription_callback(params, message);
  handler.subscription_callback(params, message);
  handler.subscription_callback(params, message);

  // Check that the message was only actually sent once
  EXPECT_EQ(messages.size(), 1);
}

}  // namespace rws

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}