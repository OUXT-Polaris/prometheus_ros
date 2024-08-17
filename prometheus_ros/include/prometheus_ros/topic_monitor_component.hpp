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

#ifndef PROMETHEUS_ROS__TOPIC_MONITOR_COMPONENT_HPP_
#define PROMETHEUS_ROS__TOPIC_MONITOR_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <topic_monitor_parameters.hpp>

namespace prometheus_ros
{
class TopicMonitorComponent : public rclcpp::Node
{
public:
  explicit TopicMonitorComponent(const rclcpp::NodeOptions & options);

private:
  const topic_monitor_node::Params parameters_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::map<std::string, std::string> current_subscribe_topics_;
  std::map<std::string, std::vector<std::string>> topic_names_and_types_;
  void updateTopicNamesAndTypes();
  void updateSubscription();
};

}  // namespace prometheus_ros

#endif  // PROMETHEUS_ROS__TOPIC_MONITOR_COMPONENT_HPP_
