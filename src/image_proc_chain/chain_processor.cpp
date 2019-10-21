#include "image_proc_chain/chain_processor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace image_proc_chain {

FlexibleChainExecutor::FlexibleChainExecutor(rclcpp::Node::SharedPtr& node, const uint32_t default_num)
    : node_(node) {
  piece_nodes_.resize(default_num);
  chain_pieces_.resize(default_num);
  for (uint32_t i = 0; i < default_num; ++i) {
    piece_nodes_[i].reset(new rclcpp::Node("chain_piece_" + std::to_string(i)));
    chain_pieces_[i].reset(new SwitchableImageProcessor(piece_nodes_[i]));
    executor_->add_node(piece_nodes_[i]);
  }
}

bool FlexibleChainExecutor::ChangeNumTo(const uint32_t num) {
  const uint32_t current_num = piece_nodes_.size();
  if (num == current_num) {
    RCLCPP_WARN(node_->get_logger(), "Requested chain-num is the same as current one(%d)", num);
    return false;
  } else if (num > current_num) {
    RCLCPP_INFO(node_->get_logger(), "Request(%d) is less than current one", num);
    const int diff = num - current_num;
    for (int i = 0; i < diff; ++i) {
      piece_nodes_.emplace_back(
          rclcpp::Node::SharedPtr(
              new rclcpp::Node("/chain_piece_" + std::to_string(i))));
      chain_pieces_.emplace_back(
          SwitchableImageProcessor::SharedPtr(
              new SwitchableImageProcessor(piece_nodes_[current_num - 1 + (i + 1)])));
      executor_->add_node((piece_nodes_[current_num + i]));
    }
    return true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Request(%d) is less than current one", num);
    const int diff = num - current_num;
    for (int i = 0; i < diff; ++i) {
      executor_->remove_node((piece_nodes_[current_num - 1 - (i + 1)]));
      chain_pieces_.pop_back();
      piece_nodes_.pop_back();
    }
    return true;
  }
}


ChainProcessor::ChainProcessor(const std::string& node_name) : node_(new rclcpp::Node(node_name)) {
  int chain_num = 3;
  node_->declare_parameter("chain_num", chain_num);

  flexible_chain_executor_.reset(new FlexibleChainExecutor(node_, chain_num));

  srv_ = node_->create_service<srv::ChangeChainNum>(
      "~/change_chain_num",
      std::bind(&ChainProcessor::ChangeNumberOfChain, this, _1, _2, _3));
}

ChainProcessor::~ChainProcessor() {
  node_->undeclare_parameter("chain_num");
}

void ChainProcessor::ChangeNumberOfChain(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<srv::ChangeChainNum::Request> request,
    std::shared_ptr<srv::ChangeChainNum::Response> response) {
  // avoid unused error
  (void)request_header;

  RCLCPP_INFO(node_->get_logger(), "Change service has called(req: %d)", request->num);

  response->successful = flexible_chain_executor_->ChangeNumTo(request->num);
}

}
