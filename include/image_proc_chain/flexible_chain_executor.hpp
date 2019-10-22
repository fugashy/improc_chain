// Copyright 2019 fugashy
#ifndef IMAGE_PROC_CHAIN__FLEXIBLE_CHAIN_EXECUTOR_HPP_
#define IMAGE_PROC_CHAIN__FLEXIBLE_CHAIN_EXECUTOR_HPP_
#include <memory>
#include <vector>

#include "image_proc_chain/chain_piece.hpp"

namespace image_proc_chain {

class FlexibleChainExecutor {
 public:
  /**
   * @brief Alias to std::shared_ptr
   */
  using SharedPtr = std::shared_ptr<FlexibleChainExecutor>;

  /**
   * @brief Initialize executor with specified index length
   *
   * @param[in] node Pointer to node
   * @param[in] index Length of index
   */
  explicit FlexibleChainExecutor(rclcpp::Node::SharedPtr& node, const uint32_t index);

  /**
   * @brief Default destructor
   */
  virtual ~FlexibleChainExecutor() = default;

  /**
   * @brief Change chain length to specified number
   *
   * 0 is not supported. It'll return false
   * The same number of currenct length is not supported. It'll return false
   *
   * @param[in] num Chain number that you want to change to.
   *
   * @return Success/Failure
   */
  bool ChangeNumTo(const uint32_t num);

  /**
   * @brief Get pointer to executor
   *
   * ex) get_executor()->spin();
   *
   * @return Pointer to executor
   */
  rclcpp::executors::SingleThreadedExecutor::SharedPtr get_executor() const { return executor_; }

 private:
  /**
   * @brief Pointer to node
   *
   * It used for logging, rearranging ChainPiece.
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief Nodes for chain_pieces_
   *
   * The length is associated with chain_pieces_
   * The length is variable by using ChangeNumTo
   */
  std::vector<rclcpp::Node::SharedPtr> piece_nodes_;

  /**
   * @brief Array of ChainPiece
   *
   * The length is variable by using ChangeNumTo
   */
  std::vector<ChainPiece::SharedPtr> chain_pieces_;

  /**
   * @brief Interprocess communication supporter
   */
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__FLEXIBLE_CHAIN_EXECUTOR_HPP_
