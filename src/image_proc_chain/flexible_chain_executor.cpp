// Copyright 2019 fugashy
#include <string>

#include "image_proc_chain/flexible_chain_executor.hpp"

namespace image_proc_chain {

FlexibleChainExecutor::FlexibleChainExecutor(
    rclcpp::Node::SharedPtr& node, const uint32_t default_num)
    : node_(node) {
  piece_nodes_.resize(default_num);
  chain_pieces_.resize(default_num);
  executor_.reset(new rclcpp::executors::SingleThreadedExecutor());
  // サービスのコールバックを受けるためには，このnode_もexecutorに登録する必要がある
  executor_->add_node(node_);
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
  RCLCPP_INFO(node_->get_logger(), "flexible_chain_executor is ready");
}

bool FlexibleChainExecutor::ChangeNumTo(const uint32_t num) {
  if (num == 0) {
    RCLCPP_WARN(node_->get_logger(), "0 num is requested, we ignore this request");
    return false;
  }

  const int current_num = piece_nodes_.size();
  if (static_cast<int>(num) == current_num) {
    RCLCPP_WARN(node_->get_logger(), "Requested chain-num is the same as current one(%d)", num);
    return false;
  } else if (static_cast<int>(num) > current_num) {
    RCLCPP_INFO(node_->get_logger(), "Request(%d) is greater than current one", num);
    const int diff = num - current_num;
    for (int i = 0; i < diff; ++i) {
      // 現在のノード数に足し合わせる
      // idとしては0数え
      // current_numは1数え
      // iは0数え
      // よって，current_num + iによって次のnode_idが計算できる
      const int node_id = current_num + i;
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
      // 現在のノード数を減らす
      // idとしては0数え
      // current_numは1数え
      // iは0数え
      // よって，current_num + i - 1によって現在のnode_idが計算できる
      const int node_id = current_num - i - 1;
      executor_->remove_node((piece_nodes_[node_id]));
      chain_pieces_.pop_back();
      piece_nodes_.pop_back();
    }
    return true;
  }
}

}  // namespace image_proc_chain
