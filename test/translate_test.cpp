#ifndef RWS__TRANSLATE_TEST_HPP_
#define RWS__TRANSLATE_TEST_HPP_

#include "rws/translate.hpp"

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/serialization.hpp"

namespace rws
{

using json = nlohmann::json;

class TranslateFixture : public testing::Test
{
public:
  TranslateFixture() {}

  void SetUp() override {}

  void TearDown() override {}

protected:
};

TEST_F(TranslateFixture, DescribeStdMsgsString)
{
  std::string msg_desc = generate_message_meta("std_msgs/msg/String");
  EXPECT_EQ(msg_desc, "string data\n");
}

TEST_F(TranslateFixture, DescribeStdMsgsBuiltins)
{
  auto msg_desc = generate_message_meta("test_msgs/msg/Builtins");

  std::string expected =
    "Duration duration_value\n"
    "Time time_value\n"
    "============\n"
    "MSG: builtin_interfaces/msg/Duration\n"
    "int32 sec\n"
    "uint32 nanosec\n"
    "============\n"
    "MSG: builtin_interfaces/msg/Time\n"
    "int32 sec\n"
    "uint32 nanosec\n";

  EXPECT_EQ(msg_desc, expected);
}

TEST_F(TranslateFixture, RosbridgeCompatibleDescriptionHasRenamedNanosecondsField)
{
  auto msg_desc = generate_message_meta("rcl_interfaces/msg/Log", true);

  std::string expected =
    "Time stamp\n"
    "uint8 level\n"
    "string name\n"
    "string msg\n"
    "string file\n"
    "string function\n"
    "uint32 line\n"
    "============\n"
    "MSG: builtin_interfaces/msg/Time\n"
    "int32 sec\n"
    "uint32 nsec\n";  // <--- renamed from nanosec to nsec

  EXPECT_EQ(msg_desc, expected);
}

TEST_F(TranslateFixture, DescribeTestMsgsUnboundedSequence)
{
  auto msg_desc = generate_message_meta("test_msgs/msg/UnboundedSequences");

  std::string expected =
    "bool[] bool_values\n"
    "uint8[] byte_values\n"
    "uint8[] char_values\n"
    "float32[] float32_values\n"
    "float64[] float64_values\n"
    "int8[] int8_values\n"
    "uint8[] uint8_values\n"
    "int16[] int16_values\n"
    "uint16[] uint16_values\n"
    "int32[] int32_values\n"
    "uint32[] uint32_values\n"
    "int64[] int64_values\n"
    "uint64[] uint64_values\n"
    "string[] string_values\n"
    "BasicTypes[] basic_types_values\n"
    "Constants[] constants_values\n"
    "Defaults[] defaults_values\n"
    "bool[] bool_values_default\n"
    "uint8[] byte_values_default\n"
    "uint8[] char_values_default\n"
    "float32[] float32_values_default\n"
    "float64[] float64_values_default\n"
    "int8[] int8_values_default\n"
    "uint8[] uint8_values_default\n"
    "int16[] int16_values_default\n"
    "uint16[] uint16_values_default\n"
    "int32[] int32_values_default\n"
    "uint32[] uint32_values_default\n"
    "int64[] int64_values_default\n"
    "uint64[] uint64_values_default\n"
    "string[] string_values_default\n"
    "int32 alignment_check\n"
    "============\n"
    "MSG: test_msgs/msg/BasicTypes\n"
    "bool bool_value\n"
    "uint8 byte_value\n"
    "uint8 char_value\n"
    "float32 float32_value\n"
    "float64 float64_value\n"
    "int8 int8_value\n"
    "uint8 uint8_value\n"
    "int16 int16_value\n"
    "uint16 uint16_value\n"
    "int32 int32_value\n"
    "uint32 uint32_value\n"
    "int64 int64_value\n"
    "uint64 uint64_value\n"
    "============\n"
    "MSG: test_msgs/msg/Constants\n"
    "============\n"
    "MSG: test_msgs/msg/Defaults\n"
    "bool bool_value\n"
    "uint8 byte_value\n"
    "uint8 char_value\n"
    "float32 float32_value\n"
    "float64 float64_value\n"
    "int8 int8_value\n"
    "uint8 uint8_value\n"
    "int16 int16_value\n"
    "uint16 uint16_value\n"
    "int32 int32_value\n"
    "uint32 uint32_value\n"
    "int64 int64_value\n"
    "uint64 uint64_value\n";

  EXPECT_EQ(msg_desc, expected);
}

TEST_F(TranslateFixture, DescribeTestMsgsArrays)
{
  auto msg_desc = generate_message_meta("test_msgs/msg/Arrays");

  std::string expected =
    "bool[3] bool_values\n"
    "uint8[3] byte_values\n"
    "uint8[3] char_values\n"
    "float32[3] float32_values\n"
    "float64[3] float64_values\n"
    "int8[3] int8_values\n"
    "uint8[3] uint8_values\n"
    "int16[3] int16_values\n"
    "uint16[3] uint16_values\n"
    "int32[3] int32_values\n"
    "uint32[3] uint32_values\n"
    "int64[3] int64_values\n"
    "uint64[3] uint64_values\n"
    "string[3] string_values\n"
    "BasicTypes[3] basic_types_values\n"
    "Constants[3] constants_values\n"
    "Defaults[3] defaults_values\n"
    "bool[3] bool_values_default\n"
    "uint8[3] byte_values_default\n"
    "uint8[3] char_values_default\n"
    "float32[3] float32_values_default\n"
    "float64[3] float64_values_default\n"
    "int8[3] int8_values_default\n"
    "uint8[3] uint8_values_default\n"
    "int16[3] int16_values_default\n"
    "uint16[3] uint16_values_default\n"
    "int32[3] int32_values_default\n"
    "uint32[3] uint32_values_default\n"
    "int64[3] int64_values_default\n"
    "uint64[3] uint64_values_default\n"
    "string[3] string_values_default\n"
    "int32 alignment_check\n"
    "============\n"
    "MSG: test_msgs/msg/BasicTypes\n"
    "bool bool_value\n"
    "uint8 byte_value\n"
    "uint8 char_value\n"
    "float32 float32_value\n"
    "float64 float64_value\n"
    "int8 int8_value\n"
    "uint8 uint8_value\n"
    "int16 int16_value\n"
    "uint16 uint16_value\n"
    "int32 int32_value\n"
    "uint32 uint32_value\n"
    "int64 int64_value\n"
    "uint64 uint64_value\n"
    "============\n"
    "MSG: test_msgs/msg/Constants\n"
    "============\n"
    "MSG: test_msgs/msg/Defaults\n"
    "bool bool_value\n"
    "uint8 byte_value\n"
    "uint8 char_value\n"
    "float32 float32_value\n"
    "float64 float64_value\n"
    "int8 int8_value\n"
    "uint8 uint8_value\n"
    "int16 int16_value\n"
    "uint16 uint16_value\n"
    "int32 int32_value\n"
    "uint32 uint32_value\n"
    "int64 int64_value\n"
    "uint64 uint64_value\n";

  EXPECT_EQ(msg_desc, expected);
}

TEST_F(TranslateFixture, DeserializeRclInterfacesLogMessage)
{
  auto log_msg = std::make_shared<rcl_interfaces::msg::Log>();
  log_msg->level = 1;
  log_msg->name = "client_handler";
  log_msg->msg =
    "process_message: "
    "{\"args\":{},\"id\":\"call_service:/rosapi/"
    "topics_and_raw_types:1\",\"op\":\"call_service\",\"service\":\"/rosapi/"
    "topics_and_raw_types\",\"type\":\"rosapi/TopicsAndRawTypes\"}";
  log_msg->file = "/src/client_handler.cpp";
  log_msg->function = "process_message";
  log_msg->line = 1;

  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();

  static rclcpp::Serialization<rcl_interfaces::msg::Log> serializer;
  serializer.serialize_message(log_msg.get(), &*serialized_msg);

  std::string expected_json_str =
    R"({"file":"/src/client_handler.cpp","function":"process_message","level":1,"line":1,"msg":"process_message: {\"args\":{},\"id\":\"call_service:/rosapi/topics_and_raw_types:1\",\"op\":\"call_service\",\"service\":\"/rosapi/topics_and_raw_types\",\"type\":\"rosapi/TopicsAndRawTypes\"}","name":"client_handler","stamp":{"nanosec":0,"sec":0}})";
  json expected = json::parse(expected_json_str);

  auto log_msg_json = rws::serialized_message_to_json("rcl_interfaces/msg/Log", serialized_msg);
  EXPECT_EQ(log_msg_json, expected);
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

#endif  // RWS__TRANSLATE_TEST_HPP_
