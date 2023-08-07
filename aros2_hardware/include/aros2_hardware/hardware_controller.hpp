#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "aros2_lifecycle/lifecycle_node.hpp"
#include "aros2_core/subscription.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace aros2_hardware
{

const aros2_lifecycle::lc_node_active_state ACTIVE_STATE_HARDWARE_ERROR = -101;

template<typename DataT>
class HardwareEntity : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  /// \brief Bind the node to the entity
  /// \param[in] node The node to bind to
  void bind_node(std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node)
  {
    node_ = node;
  }

  /// \brief Check if the imu is alive
  virtual bool is_alive() = 0;

  /// \brief Get the node
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node()
  {
    return node_.lock();
  }

  virtual std::shared_ptr<DataT> get_current_data() = 0;

private:
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

template<typename DataT, typename EntityT = HardwareEntity<DataT>>
class HardwareController : public aros2_lifecycle::LifecycleNode
{
public:
  HardwareController(const std::string & node_name)
  : aros2_lifecycle::LifecycleNode(node_name)
  {
  }

  void add_hardware_entity(std::shared_ptr<EntityT> && hardware_entity)
  {
    hardware_entity_ = hardware_entity;
    hardware_entity_->bind_node(shared_from_this());
    add_managed_entity(hardware_entity_);
  }

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    hardware_alive_timer_ = create_wall_timer(
      1s,
      [&]() {
        if (!is_hardware_alive()) {
          set_active_state(ACTIVE_STATE_HARDWARE_ERROR);
        } else {
          if (get_active_state() < 0) {
            if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
              set_active_state(aros2_lifecycle::ACTIVE_STATE_READY);
            } else {
              set_active_state(aros2_lifecycle::ACTIVE_STATE_INIT);
            }
          }
        }
      });
    return aros2_lifecycle::LifecycleNode::on_activate(state);
  }

  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    hardware_alive_timer_.reset();
    return aros2_lifecycle::LifecycleNode::on_deactivate(state);
  }

  virtual void on_destroy(const rclcpp_lifecycle::State & state) override
  {
    hardware_alive_timer_.reset();
    aros2_lifecycle::LifecycleNode::on_destroy(state);
  }

  bool is_hardware_alive()
  {
    return hardware_entity_->is_alive();
  }

  std::shared_ptr<EntityT> & get_hardware_entity()
  {
    return hardware_entity_;
  }

private:
  rclcpp::TimerBase::SharedPtr hardware_alive_timer_;
  std::shared_ptr<EntityT> hardware_entity_;
};

} // namespace aros2_hardware
