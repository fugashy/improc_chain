// Copyright 2019 fugashy
#include <memory>

#include "image_proc_chain/chain_processor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace image_proc_chain {

ChainProcessor::ChainProcessor(rclcpp::Node* node) : node_(node) {
  int chain_num = 3;
  node_->declare_parameter("chain_num", chain_num);

  flexible_chain_executor_.reset(new FlexibleChainExecutor(node_, chain_num));

  srv_ = node_->create_service<image_proc_chain_msgs::srv::ChangeChainNum>(
      "~/change_chain_num",
      std::bind(&ChainProcessor::ChangeNumberOfChain, this, _1, _2, _3));
}

ChainProcessor::~ChainProcessor() {
  node_->undeclare_parameter("chain_num");
}

void ChainProcessor::ChangeNumberOfChain(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Request> request,
    std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Response> response) {
  // avoid unused error
  (void)request_header;

  RCLCPP_INFO(node_->get_logger(), "Change service has called(req: %d)", request->num);

  response->successful = flexible_chain_executor_->ChangeNumTo(request->num);
}

}  // namespace image_proc_chain
