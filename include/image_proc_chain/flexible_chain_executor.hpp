// Copyright 2019 fugashy
#ifndef IMAGE_PROC_CHAIN__FLEXIBLE_CHAIN_EXECUTOR_HPP_
#define IMAGE_PROC_CHAIN__FLEXIBLE_CHAIN_EXECUTOR_HPP_
#include <memory>
#include <vector>

#include "image_proc_chain/chain_piece.hpp"

namespace image_proc_chain {

class FlexibleChainExecutor {
 public:
  using SharedPtr = std::shared_ptr<FlexibleChainExecutor>;

  explicit FlexibleChainExecutor(rclcpp::Node::SharedPtr& node, const uint32_t index);
  virtual ~FlexibleChainExecutor() = default;

  bool ChangeNumTo(const uint32_t num);

  rclcpp::executors::SingleThreadedExecutor::SharedPtr get_executor() const { return executor_; }

 private:
  rclcpp::Node::SharedPtr node_;

  std::vector<rclcpp::Node::SharedPtr> piece_nodes_;
  std::vector<ChainPiece::SharedPtr> chain_pieces_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__FLEXIBLE_CHAIN_EXECUTOR_HPP_
