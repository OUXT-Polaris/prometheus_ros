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
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(10s, [this]() { updateSubscription(); });
}

void TopicMonitorComponent::updateSubscription()
{
  const auto name_and_types = get_topic_names_and_types();
  for (const auto & topic : parameters_.topics) {
    if (name_and_types.find(topic) == name_and_types.end()) {
      /// Topic does not advertised.
      continue;
    }
    if (name_and_types.at(topic).empty()) {
      /// Topic type is unkown.
      continue;
    }
    if (name_and_types.at(topic).size() >= 2) {
      /// Topic type is invalid.
      continue;
    }
    if (topic_name_and_types_.find(topic) == topic_name_and_types_.end()) {
      auto callback = [&](const auto &) {};

      topic_name_and_types_.emplace(topic, name_and_types.at(topic)[0]);
      message_info_subscriptions_.emplace_back(rclcpp::create_message_info_subscription(
        get_node_topics_interface(), topic, name_and_types.at(topic)[0], rclcpp::QoS(10),
        callback));
    }
    if (topic_name_and_types_.at(topic) != name_and_types.at(topic)[0]) {
      /// Topic type is changed.
      continue;
    }
  }
}
}  // namespace prometheus_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prometheus_ros::TopicMonitorComponent)
