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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <libstatistics_collector/collector/generate_statistics_message.hpp>
#include <libstatistics_collector/topic_statistics_collector/received_message_age.hpp>
#include <libstatistics_collector/topic_statistics_collector/received_message_period.hpp>
#include <prometheus_ros/message_info_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <topic_monitor_parameters.hpp>
#include <unordered_map>

namespace prometheus_ros
{
using namespace libstatistics_collector::topic_statistics_collector;
class TopicMonitor
{
public:
  explicit TopicMonitor(const std::string & node_name);
  void onMessageReceived(
    const rclcpp::Time & source_timestamp, const rclcpp::Time & recieve_timestamp);
  const auto getMetrics(
    const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration)
    -> std::vector<statistics_msgs::msg::MetricsMessage>;
  const std::string node_name;

private:
  ReceivedMessagePeriodCollector<rclcpp::Time> period_collector_;
  ReceivedMessageAgeCollector<diagnostic_msgs::msg::DiagnosticArray> age_collector_;
};

class TopicMonitorComponent : public rclcpp::Node
{
public:
  explicit TopicMonitorComponent(const rclcpp::NodeOptions & options);
  virtual ~TopicMonitorComponent(){};

private:
  const topic_monitor_node::Params parameters_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  libstatistics_collector::topic_statistics_collector::ReceivedMessagePeriodCollector<rclcpp::Time>
    period_collector_;
  libstatistics_collector::topic_statistics_collector::ReceivedMessageAgeCollector<
    diagnostic_msgs::msg::DiagnosticArray>
    age_collector_;
  std::vector<std::shared_ptr<rclcpp::MessageInfoSubscription>> message_info_subscriptions_;
  std::unordered_map<std::string, std::string> topic_name_and_types_;
  void updateSubscription();
  void updateMetric();

  std::unordered_map<std::string, TopicMonitor> topic_monitors_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace prometheus_ros

#endif  // PROMETHEUS_ROS__TOPIC_MONITOR_COMPONENT_HPP_
