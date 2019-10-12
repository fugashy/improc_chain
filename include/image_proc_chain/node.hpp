#ifndef IMAGE_PROC_CHAIN_NODE_PROPAGATOR_HPP_
#define IMAGE_PROC_CHAIN_NODE_PROPAGATOR_HPP_
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace image_proc_chain {

class Node : public rclcpp::Node, public std::enable_shared_from_this<Node> {
 public:
  using SharedPtr = std::shared_ptr<Node>;

  explicit Node(
    const std::string& node_name,
    const std::string& namespace_ = "",
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif
