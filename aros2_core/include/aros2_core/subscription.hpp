#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace aros2_core
{

/**
 * @brief Subscription
 *
 * @tparam MessageT
 */
template<class MessageT>
class Subscription
{
public:
  /**
    * @brief Construct a new Lifecycle Subscription object
    *
    * @tparam CallbackT
    * @param node in which node the subscription is created
    * @param topic_name topic name
    * @param qos quality of service
    * @param callback callback
    * @param callback_group_type callback group type
    */
  template<typename CallbackT, typename NodeT>
  explicit Subscription(
    const NodeT & node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::CallbackGroupType callback_group_type =
    rclcpp::CallbackGroupType::MutuallyExclusive)
  : topic_name_(topic_name),
    node_base_(node->get_node_base_interface())
  {
    callback_group_ = node->create_callback_group(callback_group_type);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = callback_group_;
    subscription_ = node->template create_subscription<MessageT>(
      topic_name,
      qos,
      std::forward<CallbackT>(callback),
      opt);
  }

  /**
   * @brief Get the rclcpp subscription object
   *
   * @return rclcpp::Subscription<MessageT>::SharedPtr
   */
  typename rclcpp::Subscription<MessageT>::SharedPtr get()
  {
    return subscription_;
  }

private:
  std::string topic_name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
};

} // namespace aros2_core
