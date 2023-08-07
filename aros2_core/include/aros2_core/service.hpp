#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace aros2_core
{

/**
 * @brief Service
 *
 * @tparam ServiceT
 */
template<class ServiceT>
class Service
{
public:
  /**
   * @brief Construct a new Service object
   *
   * @tparam CallbackT
   * @param node in which node the service is created
   * @param service_name service name
   * @param callback callback
   * @param callback_group_type callback group type
   */
  template<typename CallbackT, typename NodeT>
  explicit Service(
    const NodeT & node,
    const std::string & service_name,
    CallbackT && callback,
    const rclcpp::CallbackGroupType callback_group_type =
    rclcpp::CallbackGroupType::MutuallyExclusive)
  : service_name_(service_name),
    node_base_(node->get_node_base_interface()),
    node_services_(node->get_node_services_interface())
  {
    callback_group_ = node_base_->create_callback_group(callback_group_type);
    service_ = node->template create_service<ServiceT, CallbackT>(
      service_name,
      std::forward<CallbackT>(callback),
      rclcpp::SystemDefaultsQoS().get_rmw_qos_profile(),
      callback_group_);
  }

protected:
  std::string service_name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  typename rclcpp::Service<ServiceT>::SharedPtr service_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
};

}  // namespace aros2_core
