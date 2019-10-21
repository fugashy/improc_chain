#ifndef IMAGE_PROC_CHAIN_CHAIN_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN_CHAIN_PROCESSOR_HPP_
#include "image_proc_chain/flexible_chain_executor.hpp"
#include "image_proc_chain/srv/change_chain_num.hpp"


namespace image_proc_chain {

class ChainProcessor {
 public:
  explicit ChainProcessor(rclcpp::Node::SharedPtr& node);
  virtual ~ChainProcessor();

  rclcpp::executors::SingleThreadedExecutor::SharedPtr get_executor() const {
    return flexible_chain_executor_->get_executor();
  }

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
