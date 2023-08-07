#pragma once

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace aros2_core
{

template<class MessageT>
class Publisher
{
public:
  explicit Publisher(
    const rclcpp::Node::SharedPtr & provided_node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::CallbackGroupType callback_group_type = rclcpp::CallbackGroupType::MutuallyExclusive)
  : topic_name_(topic_name), node_(provided_node)
  {
    callback_group_ = provided_node->create_callback_group(callback_group_type);
    rclcpp::PublisherOptions opt;
    opt.callback_group = callback_group_;
    publisher_ = provided_node->create_publisher<MessageT>(topic_name, qos, opt);
  }

  template<typename CallbackT>
  void add_timer(CallbackT && callback, const int qps)
  {
    timer_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto interval = 1000ms / qps;
    timer_ = node_->create_wall_timer(interval, callback, timer_callback_group_);
  }

  void publish(const MessageT & msg)
  {
    publisher_->publish(msg);
  }

  typename rclcpp::Publisher<MessageT>::SharedPtr get()
  {
    return publisher_;
  }

private:
  std::string topic_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

} // namespace aros2_core
