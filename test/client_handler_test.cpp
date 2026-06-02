
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

TEST_F(
  ClientHandlerFixture,
  subscribe_to_nonexistent_topic_succeeds_when_type_is_given)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<Connector<>>(node_interface);

  ClientHandler handler(
    0,
    node_interface,
    connector,
    true,
    [](std::string &) {},
    [](std::vector<uint8_t> &) {});

  auto json_o = json::parse(R"(
    {
      "id": "sub1",
      "op": "subscribe",
      "topic": "/topic_that_does_not_exist_yet",
      "type": "std_msgs/msg/String"
    }
  )");

  auto response = handler.process_message(json_o);

  EXPECT_EQ(response["op"], "subscribe_response");
  EXPECT_EQ(response["result"], true);
  EXPECT_EQ(response["type"], "std_msgs/msg/String");
}

TEST_F(
  ClientHandlerFixture,
  subscribe_to_nonexistent_topic_fails_when_type_is_not_given)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<Connector<>>(node_interface);

  ClientHandler handler(
    0,
    node_interface,
    connector,
    true,
    [](std::string &) {},
    [](std::vector<uint8_t> &) {});

  auto json_o = json::parse(R"(
    {
      "id": "sub1",
      "op": "subscribe",
      "topic": "/topic_that_does_not_exist_yet"
    }
  )");

  auto response = handler.process_message(json_o);

  EXPECT_EQ(response["op"], "subscribe_response");
  EXPECT_EQ(response["result"], false);
  EXPECT_TRUE(response.contains("error"));
}

TEST_F(
  ClientHandlerFixture,
  subscribe_to_nonexistent_topic_normalizes_rosbridge_style_type)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<Connector<>>(node_interface);

  ClientHandler handler( 0, node_interface, connector, true, [](std::string &) {}, [](std::vector<uint8_t> &) {});

  auto json_o = json::parse(R"(
    {
      "id": "sub1",
      "op": "subscribe",
      "topic": "/topic_that_does_not_exist_yet",
      "type": "std_msgs/String"
    }
  )");

  auto response = handler.process_message(json_o);

  EXPECT_EQ(response["op"], "subscribe_response");
  EXPECT_EQ(response["result"], true);
  EXPECT_EQ(response["type"], "std_msgs/msg/String");
}

TEST_F(
  ClientHandlerFixture,
  subscribe_to_existing_topic_fails_when_explicit_type_does_not_match)
{
  auto server_node = std::make_shared<rclcpp::Node>("server_node");
  auto node_interface = std::make_shared<NodeInterfaceImpl>(server_node);
  auto connector = std::make_shared<Connector<>>(node_interface);

  auto publisher = server_node->create_generic_publisher("/existing_topic", "test_msgs/msg/BasicTypes", rclcpp::QoS(10));

  ClientHandler handler(0, node_interface, connector, true, [](std::string &) {}, [](std::vector<uint8_t> &) {});

  auto json_o = json::parse(R"(
    {
      "id": "sub1",
      "op": "subscribe",
      "topic": "/existing_topic",
      "type": "test_msgs/msg/Strings"
    }
  )");

  auto response = handler.process_message(json_o);

  EXPECT_EQ(response["op"], "subscribe_response");
  EXPECT_EQ(response["id"], "sub1");
  EXPECT_EQ(response["result"], false);
  EXPECT_TRUE(response.contains("error"));

  EXPECT_NE(publisher, nullptr);
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