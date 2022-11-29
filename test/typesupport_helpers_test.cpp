#include "rws/typesupport_helpers.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

namespace rws
{

using rosidl_typesupport_introspection_cpp::ServiceMembers;

class TypesupportHelpersFixture : public testing::Test
{
public:
  TypesupportHelpersFixture() {}

  void SetUp() override {}

  void TearDown() override {}

protected:
};

TEST_F(TypesupportHelpersFixture, GetTypesupportHandleCanFindStandardService)
{
  std::string srv_type = "std_srvs/srv/Trigger";
  auto library = rclcpp::get_typesupport_library(srv_type, "rosidl_typesupport_introspection_cpp");
  auto ts =
    rws::get_service_typesupport_handle(srv_type, "rosidl_typesupport_introspection_cpp", *library);
  EXPECT_NE(ts, nullptr);
}

TEST_F(TypesupportHelpersFixture, GetTypeFromMessageMembers)
{
  std::string srv_type = "std_srvs/srv/Trigger";
  auto library = rclcpp::get_typesupport_library(srv_type, "rosidl_typesupport_introspection_cpp");
  auto ts =
    rws::get_service_typesupport_handle(srv_type, "rosidl_typesupport_introspection_cpp", *library);
  auto srv_members = static_cast<const ServiceMembers *>(ts->data);
  EXPECT_NE(ts, nullptr);

  auto type = rws::get_type_from_message_members(srv_members->request_members_);
  EXPECT_EQ(type, "std_srvs/srv/Trigger_Request");
}

TEST_F(TypesupportHelpersFixture, SplitNodeName)
{
  auto [ns, node_name] = rws::split_ns_node_name("/foo/bar");
  EXPECT_EQ(ns, "/foo");
  EXPECT_EQ(node_name, "bar");
}

TEST_F(TypesupportHelpersFixture, SplitNodeNameWithoutNamespace)
{
  auto [ns, node_name] = rws::split_ns_node_name("/bar");
  EXPECT_EQ(ns, "");
  EXPECT_EQ(node_name, "bar");
}

TEST_F(TypesupportHelpersFixture, GetTypesupportHandleContainServiceMembers)
{
  std::string srv_type = "std_srvs/srv/SetBool";
  auto library = rclcpp::get_typesupport_library(srv_type, "rosidl_typesupport_introspection_cpp");
  auto ts =
    rws::get_service_typesupport_handle(srv_type, "rosidl_typesupport_introspection_cpp", *library);
  EXPECT_NE(ts, nullptr);

  auto srv_members = static_cast<const ServiceMembers *>(ts->data);
  EXPECT_NE(srv_members, nullptr);

  auto request_members = srv_members->request_members_;
  EXPECT_NE(request_members, nullptr);

  auto response_members = srv_members->response_members_;
  EXPECT_NE(response_members, nullptr);

  EXPECT_EQ(request_members->member_count_, 1u);
  EXPECT_EQ(response_members->member_count_, 2u);

  EXPECT_STREQ(request_members->members_[0].name_, "data");
  EXPECT_EQ(
    request_members->members_[0].type_id_, rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL);
  EXPECT_EQ(request_members->members_[0].members_, nullptr);
  EXPECT_EQ(request_members->members_[0].array_size_, 0u);
  EXPECT_EQ(request_members->members_[0].is_upper_bound_, false);
  EXPECT_EQ(request_members->members_[0].is_array_, false);

  EXPECT_STREQ(response_members->members_[0].name_, "success");
  EXPECT_EQ(
    response_members->members_[0].type_id_, rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL);
  EXPECT_EQ(response_members->members_[0].members_, nullptr);
  EXPECT_EQ(response_members->members_[0].array_size_, 0u);
  EXPECT_EQ(response_members->members_[0].is_upper_bound_, false);
  EXPECT_EQ(response_members->members_[0].is_array_, false);

  EXPECT_STREQ(response_members->members_[1].name_, "message");
  EXPECT_EQ(
    response_members->members_[1].type_id_, rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING);
  EXPECT_EQ(response_members->members_[1].members_, nullptr);
  EXPECT_EQ(response_members->members_[1].array_size_, 0u);
  EXPECT_EQ(response_members->members_[1].is_upper_bound_, false);
  EXPECT_EQ(response_members->members_[1].is_array_, false);
}

TEST_F(TypesupportHelpersFixture, Ros1MessageTypeToRos2)
{
  auto ros1_msg_type = "std_msgs/String";
  auto ros2_msg_type = rws::message_type_to_ros2_style(ros1_msg_type);
  EXPECT_EQ(ros2_msg_type, "std_msgs/msg/String");
}

TEST_F(TypesupportHelpersFixture, Ros1MessageTypeToRos2DoesNotMutateRos2Message)
{
  auto fake_ros1_msg_type = "std_msgs/msg/String";
  auto ros1_msg_type = rws::message_type_to_ros2_style(fake_ros1_msg_type);
  EXPECT_EQ(ros1_msg_type, "std_msgs/msg/String");
}

TEST_F(TypesupportHelpersFixture, Ros2MessageTypeToRos1)
{
  auto ros2_msg_type = "std_msgs/msg/String";
  auto ros1_msg_type = rws::message_type_to_ros1_style(ros2_msg_type);
  EXPECT_EQ(ros1_msg_type, "std_msgs/String");
}

TEST_F(TypesupportHelpersFixture, Ros2MessageTypeToRos1HandleDoesNotMutateRos1MessageType)
{
  auto fake_ros2_msg_type = "std_msgs/String";
  auto ros1_msg_type = rws::message_type_to_ros1_style(fake_ros2_msg_type);
  EXPECT_EQ(ros1_msg_type, "std_msgs/String");
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
