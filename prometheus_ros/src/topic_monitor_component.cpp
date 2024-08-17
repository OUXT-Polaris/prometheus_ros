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
  const auto add_subscription = [&](const std::string & topic_name) {
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    options.topic_stats_options.publish_period = std::chrono::seconds(10);

    auto callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg) -> void {
      RCLCPP_INFO(this->get_logger(), "Received a message");
    };
    subscriptions_.emplace_back(create_generic_subscription(
      topic_name, current_subscribe_topics_[topic_name], rclcpp::QoS(10), callback, options));
  };
}
}  // namespace prometheus_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prometheus_ros::TopicMonitorComponent)