// Copyright 2019 fugashy
#include <memory>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>

#include "image_proc_chain/switchable_image_processor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace image_proc_chain {

SwitchableImageProcessor::SwitchableImageProcessor(rclcpp::Node* node) : node_(node) {
  image_processor_ = image_processors::Create(node_);

  srv_ = node_->create_service<image_proc_chain_msgs::srv::SwitchProcessorType>(
      "~/switch_processor_type",
      std::bind(&SwitchableImageProcessor::SwitchTypeOfProcessor, this, _1, _2, _3));
}

cv::Mat SwitchableImageProcessor::Process(const cv::Mat& image_in) {
  return image_processor_->Process(image_in);
}

void SwitchableImageProcessor::SwitchTypeOfProcessor(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<image_proc_chain_msgs::srv::SwitchProcessorType::Request> request,
    std::shared_ptr<image_proc_chain_msgs::srv::SwitchProcessorType::Response> response) {
  // avoid unused error
  (void)request_header;

  RCLCPP_INFO(node_->get_logger(), "Switch service has called(req: %s)", request->type.c_str());

  image_processor_.reset();
  image_processor_ = image_processors::Create(node_, request->type);
  if (image_processor_ == nullptr) {
    response->successful = false;
    return;
  }
  response->successful = true;
}

}  // namespace image_proc_chain
