// Copyright 2019 fugashy
#ifndef IMAGE_PROC_CHAIN__SWITCHABLE_IMAGE_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN__SWITCHABLE_IMAGE_PROCESSOR_HPP_
#include <memory>

#include "image_proc_chain/image_processors.hpp"
#include "image_proc_chain/srv/switch_processor_type.hpp"

namespace image_proc_chain {

class SwitchableImageProcessor {
 public:
  using SharedPtr = std::shared_ptr<SwitchableImageProcessor>;

  explicit SwitchableImageProcessor(rclcpp::Node::SharedPtr node);

  cv::Mat Process(const cv::Mat& image_in);

 private:
  void SwitchTypeOfProcessor(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<srv::SwitchProcessorType::Request> request,
      std::shared_ptr<srv::SwitchProcessorType::Response> response);

  rclcpp::Node::SharedPtr node_;

  image_processors::Base::SharedPtr image_processor_;
  rclcpp::Service<srv::SwitchProcessorType>::SharedPtr srv_;
};

}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__SWITCHABLE_IMAGE_PROCESSOR_HPP_
