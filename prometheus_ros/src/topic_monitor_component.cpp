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

void TopicMonitorComponent::updateSubscription() {}
}  // namespace prometheus_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prometheus_ros::TopicMonitorComponent)
