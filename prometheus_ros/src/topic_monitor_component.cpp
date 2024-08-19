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
#include <std_msgs/msg/header.hpp>

namespace prometheus_ros
{
TopicMonitor::TopicMonitor(const std::string & node_name) : node_name(node_name)
{
  period_collector_.Start();
  age_collector_.Start();
}

void TopicMonitor::onMessageReceived(
  const rclcpp::Time & source_timestamp, const rclcpp::Time & recieve_timestamp)
{
  period_collector_.OnMessageReceived(
    source_timestamp, static_cast<rcl_time_point_value_t>(recieve_timestamp.nanoseconds()));
  age_collector_.OnMessageReceived(
    diagnostic_msgs::build<diagnostic_msgs::msg::DiagnosticArray>()
      .header(std_msgs::build<std_msgs::msg::Header>()
                .stamp(static_cast<builtin_interfaces::msg::Time>(source_timestamp))
                .frame_id(""))
      .status({}),
    static_cast<rcl_time_point_value_t>(recieve_timestamp.nanoseconds()));
}

const auto TopicMonitor::getMetrics(
  const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration)
  -> std::vector<statistics_msgs::msg::MetricsMessage>
{
  using namespace libstatistics_collector::collector;
  return {
    GenerateStatisticMessage(
      node_name, period_collector_.GetMetricName(), period_collector_.GetMetricUnit(),
      window_end_timestamp - duration, window_end_timestamp,
      period_collector_.GetStatisticsResults()),
    GenerateStatisticMessage(
      node_name, age_collector_.GetMetricName(), age_collector_.GetMetricUnit(),
      window_end_timestamp - duration, window_end_timestamp,
      age_collector_.GetStatisticsResults())};
}

TopicMonitorComponent::TopicMonitorComponent(const rclcpp::NodeOptions & options)
: Node("topic_monitor_node", options),
  parameters_(topic_monitor_node::ParamListener(get_node_parameters_interface()).get_params())
{
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(10s, [this]() {
    updateSubscription();
    updateMetric();
  });
  period_collector_.Start();
  age_collector_.Start();
}

void TopicMonitorComponent::updateMetric()
{
  using namespace std::chrono_literals;
  const rclcpp::Time now = get_clock()->now();
  const auto period_statistic_message =
    libstatistics_collector::collector::GenerateStatisticMessage(
      get_name(), period_collector_.GetMetricName(), period_collector_.GetMetricUnit(),
      now - rclcpp::Duration(10s), now, period_collector_.GetStatisticsResults());
  // RCLCPP_INFO_STREAM(get_logger(), statistics_msgs::msg::to_yaml(period_statistic_message));
  const auto age_statistic_message = libstatistics_collector::collector::GenerateStatisticMessage(
    get_name(), age_collector_.GetMetricName(), age_collector_.GetMetricUnit(),
    now - rclcpp::Duration(10s), now, age_collector_.GetStatisticsResults());
  // RCLCPP_INFO_STREAM(get_logger(), statistics_msgs::msg::to_yaml(age_statistic_message));
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
      auto callback = [&](const auto & message_info) {
        const rclcpp::Time now = get_clock()->now();
        period_collector_.OnMessageReceived(
          rclcpp::Time(message_info.get_rmw_message_info().source_timestamp),
          static_cast<rcl_time_point_value_t>(now.nanoseconds()));
        age_collector_.OnMessageReceived(
          diagnostic_msgs::build<diagnostic_msgs::msg::DiagnosticArray>()
            .header(std_msgs::build<std_msgs::msg::Header>()
                      .stamp(static_cast<builtin_interfaces::msg::Time>(
                        rclcpp::Time(message_info.get_rmw_message_info().source_timestamp)))
                      .frame_id(""))
            .status({}),
          static_cast<rcl_time_point_value_t>(now.nanoseconds()));
      };

      RCLCPP_INFO_STREAM(get_logger(), "Start checking topic : " + topic);

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
