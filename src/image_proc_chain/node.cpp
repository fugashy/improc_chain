#include "image_proc_chain/node.hpp"


namespace image_proc_chain {

Node::Node(
    const std::string& node_name,
    const std::string& namespace_,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, namespace_, options) {
}

}
