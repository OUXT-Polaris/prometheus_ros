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

#include <prometheus_ros/topic_monitor_component.hpp>
#include <rclcpp/wait_for_message.hpp>

namespace prometheus_ros
{
TopicMonitorComponent::TopicMonitorComponent(const rclcpp::NodeOptions & options)
: Node("topic_monitor_node", options),
  parameters_(topic_monitor_node::ParamListener(get_node_parameters_interface()).get_params())
{
  //   for (const auto & topic : parameters_.topics) {
  //     subscriptions_.create_generic_subscription();
  //   }
}

void TopicMonitorComponent::updateTopicNamesAndTypes()
{
  topic_names_and_types_ = get_topic_names_and_types();
}

void TopicMonitorComponent::updateSubscription()
{
  //   const auto add_subscription = [&](const std::string & topic_name) {
  //     auto options = rclcpp::SubscriptionOptions();
  //     options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  //     options.topic_stats_options.publish_period = std::chrono::seconds(10);

  //     rclcpp::MessageInfo message_info;
  //     rclcpp::SerializedMessage msg;
  //     // rclcpp::wait_for_message<rclcpp::SerializedMessage>(
  //     //   msg, get_node_base_interface(), message_info, std::chrono::milliseconds(1000));

  //     // auto callback = [this](
  //     //                   const std::shared_ptr<rclcpp::SerializedMessage> & msg,
  //     //                   const rclcpp::MessageInfo & message_info) -> void {
  //     //   const auto now = this->now();
  //     //   // rclcpp::Time(msg->get_rcl_serialized_message().source_timestamp);
  //     //   // period_collector_.OnMessageReceived(now);
  //     //   // age_collector_.OnMessageReceived(now, msg->get_rcl_serialized_message().source_timestamp);
  //     // };
  //     // subscriptions_.emplace_back(create_generic_subscription(
  //     //   topic_name, current_subscribe_topics_[topic_name], rclcpp::QoS(10), callback, options));
  //   };
}

auto TopicMonitorComponent::getTypeSupport(const std::string & message_type_name)
  -> std::shared_ptr<rosidl_message_type_support_t>
{
  if (type_supports_.find(message_type_name) == type_supports_.end()) {
    type_supports_.emplace(
      message_type_name,
      std::shared_ptr<rosidl_message_type_support_t>(
        const_cast<rosidl_message_type_support_t *>(rclcpp::get_typesupport_handle(
          message_type_name.c_str(), "rosidl_typesupport_cpp",
          *rclcpp::get_typesupport_library(message_type_name.c_str(), "rosidl_typesupport_cpp")))));
  }
  return type_supports_.at(message_type_name);
}

rclcpp::MessageInfo TopicMonitorComponent::getMessageInfo(
  const rcl_serialized_message_t * /*serialized_msg*/, const std::string & /*message_type_name*/)
{
  // const rosidl_message_type_support_t * type_support = get_message_type_support(message_type_name);
  return rclcpp::MessageInfo();
}
}  // namespace prometheus_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prometheus_ros::TopicMonitorComponent)
