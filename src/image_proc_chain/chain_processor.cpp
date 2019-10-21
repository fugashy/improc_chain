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
    const std::string node_name = "chain_piece_" + std::to_string(i);
    piece_nodes_[i].reset(new rclcpp::Node(node_name));
    if (i == 0) {
      chain_pieces_[i].reset(new ChainPiece(piece_nodes_[i]));
    } else {
      const std::string pre_node_name = "chain_piece_" + std::to_string(i - 1);
      chain_pieces_[i].reset(new ChainPiece(piece_nodes_[i], pre_node_name + "/image_out"));
    }
    executor_->add_node(piece_nodes_[i]);
  }
}

bool FlexibleChainExecutor::ChangeNumTo(const uint32_t num) {
  const int current_num = piece_nodes_.size();
  if (static_cast<int>(num) == current_num) {
    RCLCPP_WARN(node_->get_logger(), "Requested chain-num is the same as current one(%d)", num);
    return false;
  } else if (static_cast<int>(num) > current_num) {
    RCLCPP_INFO(node_->get_logger(), "Request(%d) is less than current one", num);
    const int diff = num - current_num;
    for (int i = 0; i < diff; ++i) {
      const int node_id = current_num - 1 + (i + 1);
      const std::string node_name = "chain_piece_" + std::to_string(node_id);
      const std::string pre_node_name = "chain_piece_" + std::to_string(node_id - 1);
      piece_nodes_.emplace_back(rclcpp::Node::SharedPtr(new rclcpp::Node(node_name)));
      chain_pieces_.emplace_back(
          ChainPiece::SharedPtr(
              new ChainPiece(piece_nodes_[node_id], pre_node_name + "/image_out")));
      executor_->add_node((piece_nodes_[current_num + i]));
    }
    return true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Request(%d) is less than current one", num);
    const int diff = num - current_num;
    for (int i = 0; i < std::abs(diff); ++i) {
      const int node_id = current_num - 1 - (i + 1);
      executor_->remove_node((piece_nodes_[node_id]));
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
