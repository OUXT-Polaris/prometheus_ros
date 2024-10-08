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
#include <statistics_msgs/msg/statistic_data_type.hpp>
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

auto TopicMonitor::getPeriodMetrics(
  const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration) const
  -> statistics_msgs::msg::MetricsMessage
{
  using namespace libstatistics_collector::collector;
  return GenerateStatisticMessage(
    node_name, period_collector_.GetMetricName(), period_collector_.GetMetricUnit(),
    window_end_timestamp - duration, window_end_timestamp,
    period_collector_.GetStatisticsResults());
}

auto TopicMonitor::getAgeMetrics(
  const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration) const
  -> statistics_msgs::msg::MetricsMessage
{
  using namespace libstatistics_collector::collector;
  return GenerateStatisticMessage(
    node_name, age_collector_.GetMetricName(), age_collector_.GetMetricUnit(),
    window_end_timestamp - duration, window_end_timestamp, age_collector_.GetStatisticsResults());
}

auto TopicMonitor::getMetrics(
  const rclcpp::Time & window_end_timestamp, const rclcpp::Duration & duration) const
  -> std::vector<statistics_msgs::msg::MetricsMessage>
{
  return {
    getPeriodMetrics(window_end_timestamp, duration),
    getAgeMetrics(window_end_timestamp, duration)};
}

TopicGauge::TopicGauge(prometheus::Family<prometheus::Gauge> & family, const std::string & topic)
: average(family.Add({{"type", "average"}, {"topic", topic}})),
  min(family.Add({{"type", "min"}, {"topic", topic}})),
  max(family.Add({{"type", "max"}, {"topic", topic}})),
  std_dev(family.Add({{"type", "std_dev"}, {"topic", topic}})),
  count(family.Add({{"type", "count"}, {"topic", topic}}))
{
}

void TopicGauge::update(const statistics_msgs::msg::MetricsMessage & metrics)
{
  const auto get_value = [metrics](const std::uint8_t data_type) -> double {
    for (const auto statistic : metrics.statistics) {
      if (statistic.data_type == data_type) {
        return statistic.data;
      }
    }
    throw std::runtime_error("Metrics message was wrong, something unexpected happens.");
  };
  if (const auto value =
        get_value(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE);
      !std::isnan(value)) {
    average.Set(value);
  }
  if (const auto value =
        get_value(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM);
      !std::isnan(value)) {
    min.Set(value);
  }
  if (const auto value =
        get_value(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM);
      !std::isnan(value)) {
    max.Set(value);
  }
  if (const auto value =
        get_value(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_STDDEV);
      !std::isnan(value)) {
    std_dev.Set(value);
  }
  if (const auto value =
        get_value(statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT);
      !std::isnan(value)) {
    count.Set(value);
  }
}

TopicMonitorComponent::TopicMonitorComponent(const rclcpp::NodeOptions & options)
: Node("topic_monitor_node", options),
  parameters_(topic_monitor_node::ParamListener(get_node_parameters_interface()).get_params()),
  registry_(std::make_shared<prometheus::Registry>()),
  exposer_("0.0.0.0:" + std::to_string(parameters_.port)),
  period_gauge_family_(prometheus::BuildGauge()
                         .Name("message_period")
                         .Help("period of the ROS 2 message.")
                         .Register(*registry_)),
  age_gauge_family_(prometheus::BuildGauge()
                      .Name("message_age")
                      .Help("age of the ROS 2 message.")
                      .Register(*registry_))
{
  exposer_.RegisterCollectable(registry_);

  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, [this]() {
    updateSubscription();
    updateMetric();
  });
}

void TopicMonitorComponent::updateMetric()
{
  using namespace std::chrono_literals;
  const rclcpp::Time now = get_clock()->now();
  for (const auto & topic_monitor : topic_monitors_) {
    // for debug
    // RCLCPP_INFO_STREAM(get_logger(), "Topic : " << topic_monitor.first);
    // RCLCPP_INFO_STREAM(get_logger(), *topic_monitor.second);
    period_gauge_.at(topic_monitor.first)
      .update(topic_monitor.second->getPeriodMetrics(now, rclcpp::Duration(10s)));
    age_gauge_.at(topic_monitor.first)
      .update(topic_monitor.second->getAgeMetrics(now, rclcpp::Duration(10s)));
  }
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
        topic_monitors_.at(topic)->onMessageReceived(
          rclcpp::Time(message_info.get_rmw_message_info().source_timestamp), get_clock()->now());
      };

      RCLCPP_INFO_STREAM(get_logger(), "Start checking topic : " + topic);

      topic_name_and_types_.emplace(topic, name_and_types.at(topic)[0]);
      topic_monitors_.emplace(topic, std::make_unique<TopicMonitor>(get_name()));

      period_gauge_.emplace(topic, TopicGauge(period_gauge_family_, topic));
      age_gauge_.emplace(topic, TopicGauge(age_gauge_family_, topic));

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
