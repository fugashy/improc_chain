#ifndef IMAGE_PROC_CHAIN_CHAIN_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN_CHAIN_PROCESSOR_HPP_

#include "image_proc_chain/switchable_image_processor.hpp"
#include "image_proc_chain/srv/change_chain_num.hpp"


namespace image_proc_chain {

class FlexibleChainExecutor {
 public:
  using SharedPtr = std::shared_ptr<FlexibleChainExecutor>;

  explicit FlexibleChainExecutor(rclcpp::Node::SharedPtr& node, const uint32_t index);
  virtual ~FlexibleChainExecutor() = default;

  bool ChangeNumTo(const uint32_t num);

 private:
  rclcpp::Node::SharedPtr node_;

  std::vector<rclcpp::Node::SharedPtr> piece_nodes_;
  std::vector<SwitchableImageProcessor::SharedPtr> chain_pieces_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};


class ChainProcessor {
 public:
  explicit ChainProcessor(const std::string& node_name);
  virtual ~ChainProcessor();

 private:

  void ChangeNumberOfChain(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<srv::ChangeChainNum::Request> request,
      std::shared_ptr<srv::ChangeChainNum::Response> response);

  rclcpp::Node::SharedPtr node_;
  FlexibleChainExecutor::SharedPtr flexible_chain_executor_;

  rclcpp::Service<srv::ChangeChainNum>::SharedPtr srv_;
};

}

#endif
