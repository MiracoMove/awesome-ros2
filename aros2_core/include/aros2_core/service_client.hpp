#pragma once

#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace aros2_core
{

/**
 * @brief Service client
 *
 * @tparam ServiceT
 */
template<class ServiceT>
class ServiceClient
{
public:
  /**
   * @brief Construct a new Service Client object
   *
   * @param node in which node the service client is created
   * @param service_name service name
   * @param callback_group_type callback group type
   */
  template<typename NodeT>
  explicit ServiceClient(
    const NodeT & node,
    const std::string & service_name,
    const rclcpp::CallbackGroupType callback_group_type =
    rclcpp::CallbackGroupType::MutuallyExclusive)
  : service_name_(service_name),
    node_base_(node->get_node_base_interface()),
    node_logging_(node->get_node_logging_interface()),
    node_graph_(node->get_node_graph_interface()),
    node_services_(node->get_node_services_interface())
  {
    callback_group_ = node_base_->create_callback_group(
      callback_group_type,
      false);
    callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    callback_group_executor_->add_callback_group(
      callback_group_,
      node_base_);
    client_ = node->template create_client<ServiceT>(
      service_name,
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);
  }

  /**
   * @brief Get the service name object
   *
   * @return std::string
   */
  std::string get_service_name()
  {
    return service_name_;
  }

  /**
   * @brief Invoke service
   *
   * @param request request
   * @param timeout timeout
   * @return ServiceT::Response::SharedPtr
   */
  typename ServiceT::Response::SharedPtr invoke(
    typename ServiceT::Request::SharedPtr & request,
    const std::chrono::nanoseconds timeout = -1ns)
  {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                get_service_name() + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO_STREAM(
        node_logging_->get_logger(),
        get_service_name() << " service client: waiting for service to appear...");
    }

    auto future_result = client_->async_send_request(request);

    if (callback_group_executor_->spin_until_future_complete(future_result, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      client_->remove_pending_request(future_result);
      throw std::runtime_error(get_service_name() + " service client: async_send_request failed");
    }

    return future_result.get();
  }

  /**
   * @brief Wait for service
   *
   * @param timeout timeout
   * @return true
   * @return false
   */
  bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    return client_->wait_for_service(timeout);
  }

protected:
  std::string service_name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
};

}  // namespace aros2_core
