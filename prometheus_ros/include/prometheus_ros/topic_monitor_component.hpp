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

#include <prometheus/counter.h>
#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>

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
  auto getPeriodMetrics(
    const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration) const
    -> statistics_msgs::msg::MetricsMessage;
  auto getAgeMetric(const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration)
    const -> statistics_msgs::msg::MetricsMessage;
  auto getMetrics(const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration)
    const -> std::vector<statistics_msgs::msg::MetricsMessage>;
  const std::string node_name;

  friend std::ostream & operator<<(std::ostream & os, const TopicMonitor & obj);

private:
  ReceivedMessagePeriodCollector<rclcpp::Time> period_collector_;
  ReceivedMessageAgeCollector<diagnostic_msgs::msg::DiagnosticArray> age_collector_;
};

std::ostream & operator<<(std::ostream & os, const TopicMonitor & obj)
{
  os << obj.period_collector_.GetStatusString() << "\n";
  os << obj.age_collector_.GetStatusString();
  return os;
}

struct TopicGauge
{
  TopicGauge(prometheus::Family<prometheus::Gauge> & family);
  prometheus::Gauge & average;
  prometheus::Gauge & min;
  prometheus::Gauge & max;
  prometheus::Gauge & std_dev;
  prometheus::Gauge & count;
};

class TopicMonitorComponent : public rclcpp::Node
{
public:
  explicit TopicMonitorComponent(const rclcpp::NodeOptions & options);
  virtual ~TopicMonitorComponent(){};

private:
  const topic_monitor_node::Params parameters_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::vector<std::shared_ptr<rclcpp::MessageInfoSubscription>> message_info_subscriptions_;
  std::unordered_map<std::string, std::string> topic_name_and_types_;
  void updateSubscription();
  void updateMetric();

  std::unordered_map<std::string, std::unique_ptr<TopicMonitor>> topic_monitors_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<prometheus::Registry> registry_;
  prometheus::Exposer exposer_;
  prometheus::Family<prometheus::Gauge> & period_gauge_family_;
  //   std::unordered_map<std::string, std::unordered_map<std::string, prometheus::Gauge &>>
  //     period_gauges_;
  prometheus::Family<prometheus::Gauge> & age_gauge_family_;
  // std::unordered_map<std::string, std::unordered_map<std::string, prometheus::Gauge &>> age_gauges_;
};

}  // namespace prometheus_ros

#endif  // PROMETHEUS_ROS__TOPIC_MONITOR_COMPONENT_HPP_
