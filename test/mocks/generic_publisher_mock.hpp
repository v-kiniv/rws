#ifndef GENERIC_PUBLISHER_MOCK_H__
#define GENERIC_PUBLISHER_MOCK_H__


#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

class GenericPublisherMock
{
public:
  MOCK_METHOD(void, publish, (const rclcpp::SerializedMessage message), ());
};

#endif // GENERIC_PUBLISHER_MOCK_H__