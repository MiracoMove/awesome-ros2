#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>

// ROS2 相关依赖
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

// aros2_core 相关依赖
#include "aros2_core/service_client.hpp"
#include "aros2_core/subscription.hpp"

using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using TransitionEvent = lifecycle_msgs::msg::TransitionEvent;

namespace aros2_lifecycle
{

class LifecycleStateManager
{
public:
  template<typename NodeT>
  LifecycleStateManager(
    const std::string managed_node_name,
    const NodeT & parent_node)
  : managed_node_name_(managed_node_name),
    node_logging_(parent_node->get_node_logging_interface())
  {
    change_state_srv_ = std::make_shared<aros2_core::ServiceClient<ChangeState>>(
      parent_node,
      managed_node_name_ + "/change_state");
    get_state_srv_ = std::make_shared<aros2_core::ServiceClient<GetState>>(
      parent_node,
      managed_node_name_ + "/get_state");
    transition_event_sub_ = std::make_shared<aros2_core::Subscription<TransitionEvent>>(
      parent_node,
      managed_node_name_ + "/transition_event",
      rclcpp::QoS{10},
      std::bind(&LifecycleStateManager::transition_event_callback, this, std::placeholders::_1));
  }

  // 状态管理
  bool configure()
  {
    RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Configuring " << managed_node_name_);
    return call_change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  }
  bool activate()
  {
    RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Activating " << managed_node_name_);
    return call_change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }
  bool deactivate()
  {
    RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Deactivating " << managed_node_name_);
    return call_change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }
  bool cleanup()
  {
    RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Cleaning up " << managed_node_name_);
    return call_change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  }
  bool shutdown()
  {
    RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Shutting down " << managed_node_name_);
    return call_change_state(lifecycle_msgs::msg::Transition::TRANSITION_DESTROY);
  }

  // 状态查询
  bool is_unconfigured()
  {
    return is_in_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
  bool is_active()
  {
    return is_in_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
  bool is_inactive()
  {
    return is_in_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }
  bool is_shutdown()
  {
    return is_in_state(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  }
  lifecycle_msgs::msg::State get_state()
  {
    auto req = std::make_shared<GetState::Request>();
    auto res = get_state_srv_->invoke(req, 1s);
    return res->current_state;
  }

private:
  bool call_change_state(uint8_t state_id)
  {
    auto req = std::make_shared<ChangeState::Request>();
    req->transition.id = state_id;
    try {
      auto res = change_state_srv_->invoke(req, 1s);
      return res->success;
    } catch (const std::exception & e) {
      return false;
    }
    return true;
  }

  bool is_in_state(uint8_t state_id)
  {
    try {
      auto state = get_state();
      return state.id == state_id;
    } catch (const std::exception & e) {
      return false;
    }
  }

  void transition_event_callback(const TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM(
      node_logging_->get_logger(),
      "Transition event: " << managed_node_name_ << ": " << msg->start_state.label << "," <<
        msg->goal_state.label);
  }


  std::string managed_node_name_;

  std::shared_ptr<aros2_core::ServiceClient<ChangeState>> change_state_srv_;
  std::shared_ptr<aros2_core::ServiceClient<GetState>> get_state_srv_;
  std::shared_ptr<aros2_core::Subscription<TransitionEvent>> transition_event_sub_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
};

} // namespace aros2_lifecycle
