
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

class GenericPublisherMock
{
public:
  MOCK_METHOD(void, publish, (const rclcpp::SerializedMessage message), ());
};