#include "image_proc_chain/switchable_image_processor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace image_proc_chain {

SwitchableImageProcessor::SwitchableImageProcessor(rclcpp::Node::SharedPtr node) : node_(node) {
  image_processor_ = image_processors::Create(node_);

  srv_ = node_->create_service<srv::SwitchProcessorType>(
      "~/switch_processor_type",
      std::bind(&SwitchableImageProcessor::SwitchTypeOfProcessor, this, _1, _2, _3));
}

cv::Mat SwitchableImageProcessor::Process(const cv::Mat& image_in) {
  return image_processor_->Process(image_in);
}

void SwitchableImageProcessor::SwitchTypeOfProcessor(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<srv::SwitchProcessorType::Request> request,
    std::shared_ptr<srv::SwitchProcessorType::Response> response) {
  // avoid unused error
  (void)request_header;

  RCLCPP_INFO(node_->get_logger(), "Switch service has called(req: %s)", request->type.c_str());
  node_->declare_parameter("type", request->type);
  try {
    image_processor_.reset();
    image_processor_ = image_processors::Create(node_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    response->successful = false;
    return;
  }
  response->successful = true;
}


}
