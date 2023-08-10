#pragma once

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int8.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "aros2_lifecycle/lifecycle_publisher.hpp"

using namespace std::chrono_literals;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace aros2_lifecycle
{

using lc_node_active_state = int8_t;
const lc_node_active_state ACTIVE_STATE_ERROR = -1;
const lc_node_active_state ACTIVE_STATE_INIT = 0;
const lc_node_active_state ACTIVE_STATE_READY = 1;

/**
 * @brief Lifecycle node wrapper
 *
 */
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct a new Lifecycle Node object
   *
   * @param node_name node name
   * @param options node options
   */
  explicit LifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode(
      node_name,
      "",
      options)
  {
  }

  /**
   * @brief Construct a new Lifecycle Node object
   *
   * @param node_name node name
   * @param namespace_ namespace
   * @param options node options
   */
  LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode(
      node_name,
      namespace_,
      options,
      true
  )
  {
    register_on_configure(
      std::bind(
        &LifecycleNode::inner_on_configure, this,
        std::placeholders::_1));
    register_on_activate(std::bind(&LifecycleNode::inner_on_activate, this, std::placeholders::_1));
    register_on_deactivate(
      std::bind(&LifecycleNode::inner_on_deactivate, this, std::placeholders::_1));
    register_on_cleanup(std::bind(&LifecycleNode::inner_on_cleanup, this, std::placeholders::_1));
    register_on_shutdown(std::bind(&LifecycleNode::inner_on_shutdown, this, std::placeholders::_1));
    register_on_error(std::bind(&LifecycleNode::inner_on_error, this, std::placeholders::_1));
  }

  /**
   * @brief Destroy the MMLifecycleNode object
   *
   */
  ~LifecycleNode()
  {
    inner_on_shutdown(get_current_state());
  }

  /**
   * @brief Set node active state
   *
   * @param state
   */
  void set_active_state(lc_node_active_state state)
  {
    active_state_ = state;
  }

  /**
   * @brief Get the active state
   *
   * @return int8_t
   */
  lc_node_active_state get_active_state()
  {
    return active_state_;
  }

  /**
   * @brief Check if the node active_state is active
   *
   * @return true
   * @return false
   */
  bool is_active()
  {
    return active_state_ > 0;
  }

  /**
   * @brief Set the active state publish qps
   *
   * @param qps
   */
  void set_active_state_publish_qps(double qps)
  {
    // TODO: need to add lock here
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
      active_state_publish_qps_ != qps)
    {
      active_state_pub_->remove_timer();
      active_state_pub_->add_timer(
        std::bind(&LifecycleNode::active_state_timer_callback, this),
        active_state_publish_qps_);
      active_state_publish_qps_ = qps;
    }
  }

  /**
   * @brief to do something when configure
   *
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & state)
  {
    (void)state;
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief to do something when activate
   *
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    (void)state;
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief to do something when deactivate
   *
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    (void)state;
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief to do something when cleanup
   *
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_error(const rclcpp_lifecycle::State & state)
  {
    (void)state;
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief to do something when cleanup, used for cleanup and shutdown
   *
   * @param state
   */
  virtual void on_destroy(const rclcpp_lifecycle::State & state)
  {
    (void)state;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) final
  {
    (void)state;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) final
  {
    (void)state;
    return CallbackReturn::SUCCESS;
  }

private:
  void active_state_timer_callback()
  {
    auto msg = std::make_shared<std_msgs::msg::Int8>();
    msg->data = active_state_;
    active_state_pub_->publish(std::move(*msg));
  }
  CallbackReturn inner_on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "on_configure() is called. Current state is " << state.label());
    // 初始化运行状态发布器
    active_state_pub_ =
      std::make_shared<LifecyclePublisher<std_msgs::msg::Int8>>(
      shared_from_this(),
      "~/active_state",
      rclcpp::SensorDataQoS());
    auto ret = on_configure(state);
    if (ret != CallbackReturn::SUCCESS) {
      return ret;
    }
    RCLCPP_INFO_STREAM(get_logger(), "on_configure() is finished.");
    return ret;
  }
  CallbackReturn inner_on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "on_activate() is called. Current state is " << state.label());

    // 配置运行状态发布器
    active_state_pub_->add_timer(
      std::bind(&LifecycleNode::active_state_timer_callback, this),
      active_state_publish_qps_);
    auto ret = on_activate(state);
    if (ret != CallbackReturn::SUCCESS) {
      return ret;
    }
    set_active_state(ACTIVE_STATE_READY);
    rclcpp_lifecycle::LifecycleNode::on_activate(state);
    RCLCPP_INFO_STREAM(get_logger(), "on_activate() is finished.");
    return ret;
  }
  CallbackReturn inner_on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "on_deactivate() is called. Current state is " << state.label());
    rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    auto ret = on_deactivate(state);
    active_state_pub_->remove_timer();
    set_active_state(ACTIVE_STATE_INIT);
    RCLCPP_INFO_STREAM(get_logger(), "on_deactivate() is finished.");
    return ret;
  }
  CallbackReturn inner_on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "on_cleanup() is called. Current state is " << state.label());
    destroy(state);
    RCLCPP_INFO_STREAM(get_logger(), "on_cleanup() is finished.");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn inner_on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "on_shutdown() is called. Current state is " << state.label());
    destroy(state);
    RCLCPP_INFO_STREAM(get_logger(), "on_shutdown() is finished.");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn inner_on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "on_error() is called. Current state is " << state.label());
    auto ret = on_error(state);
    destroy(state);
    RCLCPP_INFO_STREAM(get_logger(), "on_error() is finished.");
    return ret;
  }
  void destroy(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO_STREAM(get_logger(), "destroy() is called.");
    on_destroy(state);
    if (active_state_pub_ != nullptr) {
      active_state_pub_->remove_timer();
    }
    active_state_pub_.reset();
    set_active_state(ACTIVE_STATE_INIT);
  }

  lc_node_active_state active_state_{ACTIVE_STATE_INIT};
  double active_state_publish_qps_{1};
  std::shared_ptr<LifecyclePublisher<std_msgs::msg::Int8>> active_state_pub_;
};

} // namespace aros2_lifecycle
