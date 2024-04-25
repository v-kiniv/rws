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

#include <cstdio>
#include <nlohmann/json.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rws/client_handler.hpp"
#include "rws/connector.hpp"
#include "rws/node_interface_impl.hpp"

using json = nlohmann::json;
using websocketpp::connection_hdl;
using websocketpp::lib::bind;
using websocketpp::lib::condition_variable;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
typedef websocketpp::server<websocketpp::config::asio> server;

enum action_type {
  SUBSCRIBE,
  UNSUBSCRIBE,
  DROP,
  MESSAGE,
  BINARY_REPLY,
  TEXT_REPLY,
};

struct action
{
  action(action_type t, connection_hdl h) : type(t), hdl(h) {}
  action(action_type t, connection_hdl h, server::message_ptr m) : type(t), hdl(h), msg(m) {}
  action(action_type t, connection_hdl h, std::vector<std::uint8_t> br)
  : type(t), hdl(h), binary_reply(br)
  {
  }
  action(action_type t, connection_hdl h, std::string tr) : type(t), hdl(h), text_reply(tr) {}

  action_type type;
  websocketpp::connection_hdl hdl;
  server::message_ptr msg;
  std::vector<std::uint8_t> binary_reply;
  std::string text_reply;
};

struct connection_data
{
  int client_id;
  std::shared_ptr<rws::ClientHandler> node;
  bool is_alive;
};

class ServerNode : public rclcpp::Node
{
public:
  ServerNode() : Node("rws_server"), node_interface_(nullptr), connector_(nullptr)
  {
    rosbridge_compatible_ = this->declare_parameter("rosbridge_compatible", true);
    port_ = this->declare_parameter("port", 9090);
    bool watchdog = this->declare_parameter("watchdog", true);

    RCLCPP_INFO(get_logger(), "RWS start listening on port %d", port_);

    if (watchdog) {
      ping_timer_ = this->create_wall_timer(
        std::chrono::seconds(6), std::bind(&ServerNode::ping_all_clients, this));
    }

    // set up access channels to only log interesting things
    endpoint_.clear_access_channels(websocketpp::log::alevel::all);
    endpoint_.set_access_channels(websocketpp::log::alevel::access_core);
    endpoint_.set_access_channels(websocketpp::log::alevel::app);

    // Initialize the Asio transport policy
    endpoint_.init_asio();

    // Bind the handlers we are using
    endpoint_.set_open_handler(bind(&ServerNode::on_open, this, _1));
    endpoint_.set_close_handler(bind(&ServerNode::on_close, this, _1));
    endpoint_.set_message_handler(bind(&ServerNode::on_message, this, _1, _2));
    endpoint_.set_pong_timeout_handler(bind(&ServerNode::on_pong_timeout, this, _1));
    endpoint_.set_reuse_addr(true);

    server_thread_ = std::thread(bind(&ServerNode::run, this));
  }

  void run()
  {
    running_ = true;

    try {
      endpoint_.listen(port_);
      endpoint_.start_accept();

      processing_thread_ = std::thread(bind(&ServerNode::process_messages, this));

      endpoint_.run();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Exception while running websocket server: %s", e.what());
    }
  }

  void shutdown()
  {
    if (!running_) {
      return;
    }
    running_ = false;
    con_list::iterator it;
    for (it = connections_.begin(); it != connections_.end(); ++it) {
      endpoint_.pause_reading(it->first);
      endpoint_.close(
        it->first, websocketpp::close::status::going_away, "Server is shutting down.");
    }

    endpoint_.stop_listening();

    // Notify to unlock message processing thread
    action_cond_.notify_one();

    server_thread_.join();
    processing_thread_.join();
  }

  void process_messages()
  {
    while (running_) {
      std::unique_lock<std::mutex> lock(action_lock_);

      while (actions_.empty()) {
        if (!running_) {
          lock.unlock();
          return;
        }
        action_cond_.wait(lock);
      }

      action a = actions_.front();
      actions_.pop();

      lock.unlock();

      connection_data * cd = get_con_data(a.hdl);

      if (a.type == SUBSCRIBE) {
        std::lock_guard<std::mutex> guard(connection_lock_);
        connections_[a.hdl] = subscribe(a);
      } else if (a.type == UNSUBSCRIBE) {
        std::lock_guard<std::mutex> guard(connection_lock_);
        RCLCPP_INFO(get_logger(), "Closing connection with client_id %d", cd->client_id);

        connections_.erase(a.hdl);
      } else if (a.type == DROP) {
        try {
          cd->is_alive = false;
          endpoint_.close(a.hdl, websocketpp::close::status::policy_violation, "No pong received");
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Failed to close connection: %s", e.what());
        }
      } else if (a.type == MESSAGE) {
        std::lock_guard<std::mutex> guard(connection_lock_);

        send_message_to_node(a);
      } else if (a.type == TEXT_REPLY && cd && cd->is_alive) {
        std::lock_guard<std::mutex> guard(connection_lock_);
        try {
          this->endpoint_.send(a.hdl, a.text_reply, websocketpp::frame::opcode::text);
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Failed to send string reply: %s", e.what());
        }
      } else if (a.type == BINARY_REPLY && cd && cd->is_alive) {
        std::lock_guard<std::mutex> guard(connection_lock_);
        try {
          this->endpoint_.send(
            a.hdl, a.binary_reply.data(), a.binary_reply.size(),
            websocketpp::frame::opcode::binary);
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Failed to send binary reply: %s", e.what());
        }
      }
    }
  }

private:
  typedef std::map<connection_hdl, connection_data, std::owner_less<connection_hdl>> con_list;

  bool rosbridge_compatible_;
  bool running_ = false;
  con_list connections_;
  condition_variable action_cond_;
  rclcpp::TimerBase::SharedPtr ping_timer_;
  server endpoint_;
  size_t next_client_id_ = 0;
  std::queue<action> actions_;
  uint16_t port_;
  std::mutex action_lock_;
  std::mutex connection_lock_;
  std::thread server_thread_;
  std::thread processing_thread_;
  std::shared_ptr<rws::NodeInterfaceImpl> node_interface_;
  std::shared_ptr<rws::Connector<>> connector_;

  connection_data * get_con_data(connection_hdl & hdl)
  {
    auto it = connections_.find(hdl);

    if (it == connections_.end()) {
      return nullptr;
    }

    return &it->second;
  }

  std::shared_ptr<rws::NodeInterfaceImpl> get_node_interface()
  {
    if (!node_interface_) {
      node_interface_ = std::make_shared<rws::NodeInterfaceImpl>(this->shared_from_this());
    }

    return node_interface_;
  }

  std::shared_ptr<rws::Connector<>> get_connector()
  {
    if (!connector_) {
      connector_ = std::make_shared<rws::Connector<>>(get_node_interface());
    }

    return connector_;
  }

  connection_data subscribe(action & a)
  {
    connection_data data;
    data.client_id = next_client_id_++;
    data.is_alive = true;
    data.node = std::make_shared<rws::ClientHandler>(
      data.client_id, get_node_interface(), get_connector(), rosbridge_compatible_,
      [this, a](std::string & msg) {
        {
          websocketpp::lib::lock_guard<websocketpp::lib::mutex> guard(action_lock_);
          actions_.push(action(TEXT_REPLY, a.hdl, msg));
        }
        action_cond_.notify_one();
      },
      [this, a](std::vector<std::uint8_t> & msg) {
        {
          websocketpp::lib::lock_guard<websocketpp::lib::mutex> guard(action_lock_);
          actions_.push(action(BINARY_REPLY, a.hdl, msg));
        }
        action_cond_.notify_one();
      });

    return data;
  }

  void send_message_to_node(action & a)
  {
    connection_data * cd = get_con_data(a.hdl);

    auto client_node = cd->node;
    try {
      auto json_msg = json::parse(a.msg->get_payload());
      auto response = client_node->process_message(json_msg);
      std::string response_str = response.dump();

      {
        std::lock_guard<std::mutex> guard(action_lock_);
        actions_.push(action(TEXT_REPLY, a.hdl, response_str));
      }
      action_cond_.notify_one();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to parse JSON: %s", e.what());
    }
  }

  void on_open(connection_hdl hdl)
  {
    {
      std::lock_guard<std::mutex> guard(action_lock_);
      actions_.push(action(SUBSCRIBE, hdl));
    }
    action_cond_.notify_one();
  }

  void on_close(connection_hdl hdl)
  {
    {
      std::lock_guard<std::mutex> guard(action_lock_);
      actions_.push(action(UNSUBSCRIBE, hdl));
    }
    action_cond_.notify_one();
  }

  void on_message(connection_hdl hdl, server::message_ptr msg)
  {
    {
      std::lock_guard<std::mutex> guard(action_lock_);
      actions_.push(action(MESSAGE, hdl, msg));
    }
    action_cond_.notify_one();
  }

  void on_pong_timeout(connection_hdl hdl)
  {
    if(hdl.expired() || get_con_data(hdl) == nullptr) {
      // TCP connection dropped before timeout and the client is already disposed
      return;
    }

    RCLCPP_WARN(get_logger(), "Pong timeout");
    {
      std::lock_guard<std::mutex> guard(action_lock_);
      actions_.push(action(DROP, hdl));
    }
    action_cond_.notify_one();
  }

  void ping_all_clients()
  {
    for (auto & connection : connections_) {
      try {
        this->endpoint_.ping(connection.first, "");
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to send ping: %s", e.what());
      }
    }
  }
};

// Declare as global so it's accessible inside the signal handler
std::shared_ptr<ServerNode> g_server_;

void signal_handler(int sig) { (void)sig; g_server_->shutdown(); }

int main(int argc, char * argv[])
{
  if (strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") == 0) {
    std::cout << "\033[1;31mUse rmw_fastrtps_dynamic_cpp instead of "
              << "rmw_fastrtps_cpp as RMW implementation.\033[0m" << std::endl;
    return -1;
  }

  signal(SIGINT, signal_handler);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  g_server_ = std::make_shared<ServerNode>();
  executor.add_node(g_server_);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}