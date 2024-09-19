
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
      ti, node_interface, connector, true, [](std::string & msg) {},
      [](std::vector<std::uint8_t> & msg) {}));
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
      ti, node_interface, connector, false, [](std::string & msg) {},
      [](std::vector<std::uint8_t> & msg) {}));
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
      ti, node_interface, connector, true, [](std::string & msg) {},
      [](std::vector<std::uint8_t> & msg) {}));
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

}  // namespace rws

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}