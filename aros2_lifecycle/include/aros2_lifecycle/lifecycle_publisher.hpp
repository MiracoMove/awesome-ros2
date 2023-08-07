#pragma once

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/create_timer.hpp"

using namespace std::chrono_literals;

namespace aros2_lifecycle
{

/**
 * @brief Lifecycle publisher
 *
 * @tparam MessageT
 */
template<class MessageT>
class LifecyclePublisher
{
public:
  /** @brief Create lifecycle publisher
   * @param[in] node in which node the publisher is created
   * @param[in] topic_name topic name
   * @param[in] qos quality of service
   * @param[in] callback_group_type callback group type
   */
  explicit LifecyclePublisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::CallbackGroupType callback_group_type = rclcpp::CallbackGroupType::MutuallyExclusive)
  : topic_name_(topic_name),
    node_base_(node->get_node_base_interface()),
    node_timers_(node->get_node_timers_interface())
  {
    callback_group_ = node->create_callback_group(callback_group_type);
    rclcpp::PublisherOptions opt;
    opt.callback_group = callback_group_;
    publisher_ = node->create_publisher<MessageT>(topic_name, qos, opt);
  }

  /**
   * @brief Destroy the Lifecycle Publisher object
   *
   */
  ~LifecyclePublisher()
  {
    if (timer_ != nullptr) {
      remove_timer();
    }
  }

  /**
   * @brief Add timer
   *
   * @tparam CallbackT
   * @param[in] callback callback function when timer is triggered
   * @param[in] qps frequency of the timer
   */
  template<typename CallbackT>
  void add_timer(CallbackT && callback, const double qps)
  {
    timer_callback_group_ = node_base_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto interval = 1000ms / qps;
    timer_ = rclcpp::create_wall_timer(
      interval,
      std::forward<CallbackT>(callback),
      timer_callback_group_,
      node_base_.get(),
      node_timers_.get());
  }

  /**
   * @brief Remove timer
   *
   */
  void remove_timer()
  {
    if (timer_ != nullptr) {
      timer_->cancel();
    }
    timer_.reset();
    timer_callback_group_.reset();
  }

  /**
   * @brief Publish message
   *
   * @param msg
   */
  void publish(const MessageT & msg)
  {
    publisher_->publish(msg);
  }

  /**
   * @brief Check if the publisher is activated
   *
   * @return true
   * @return false
   */
  bool is_activated()
  {
    return publisher_->is_activated();
  }

  /**
   * @brief Get the rclcpp publisher object
   *
   * @return rclcpp::Publisher<MessageT>::SharedPtr
   */
  typename rclcpp::Publisher<MessageT>::SharedPtr get()
  {
    return publisher_;
  }

private:
  std::string topic_name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

} // namespace aros2_lifecycle
