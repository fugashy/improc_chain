#ifndef IMAGE_PROC_CHAIN_MANAGER_HPP_
#define IMAGE_PROC_CHAIN_MANAGER_HPP_
#include <memory>

#include <image_proc_chain_msgs/srv/change_chain_num.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

namespace image_proc_chain {

class ComponentManager : public rclcpp_components::ComponentManager {
 public:
  explicit ComponentManager(
      std::weak_ptr<rclcpp::Executor> executor = std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
      std::string node_name = "ComponentManager",
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        .start_parameter_services(false)
        .start_parameter_event_publisher(false));
  ~ComponentManager() = default;

 private:
  void ChangeChainNum(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Request> request,
      std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Response> response);

  rclcpp::Service<image_proc_chain_msgs::srv::ChangeChainNum>::SharedPtr srv_;
};

}  // end of namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN_MANAGER_HPP_
