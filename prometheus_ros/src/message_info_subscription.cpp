// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
// Copyright (c) 2024 OUXT Polaris
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

#include <memory>
#include <prometheus_ros/message_info_subscription.hpp>
#include <string>

#include "rcl/subscription.h"
#include "rclcpp/exceptions.hpp"

namespace rclcpp
{

std::shared_ptr<void> MessageInfoSubscription::create_message()
{
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage> MessageInfoSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>(0);
}

void MessageInfoSubscription::handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
    "handle_message is not implemented for MessageInfoSubscription");
}

void MessageInfoSubscription::handle_serialized_message(
  const std::shared_ptr<rclcpp::SerializedMessage> &, const rclcpp::MessageInfo & message_info)
{
  callback_(message_info);
}

void MessageInfoSubscription::handle_loaned_message(
  void * message, const rclcpp::MessageInfo & message_info)
{
  (void)message;
  (void)message_info;
  throw rclcpp::exceptions::UnimplementedError(
    "handle_loaned_message is not implemented for MessageInfoSubscription");
}

void MessageInfoSubscription::return_message(std::shared_ptr<void> & message)
{
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  return_serialized_message(typed_message);
}

void MessageInfoSubscription::return_serialized_message(
  std::shared_ptr<rclcpp::SerializedMessage> & message)
{
  message.reset();
}

}  // namespace rclcpp
