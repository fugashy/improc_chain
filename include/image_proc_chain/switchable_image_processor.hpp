#ifndef IMAGE_PROC_CHAIN_SWITCHABLE_IMAGE_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN_SWITCHABLE_IMAGE_PROCESSOR_HPP_

#include "image_proc_chain/image_processors.hpp"
#include "image_proc_chain/srv/switch_processor_type.hpp"

namespace image_proc_chain {

class SwitchableImageProcessor {
 public:
  explicit SwitchableImageProcessor(rclcpp::Node::SharedPtr node);

 private:
  void SwitchTypeOfProcessor(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<srv::SwitchProcessorType::Request> request,
      std::shared_ptr<srv::SwitchProcessorType::Response> response);

  rclcpp::Node::SharedPtr node_;

  image_processors::Base::SharedPtr image_processor_;
  rclcpp::Service<srv::SwitchProcessorType>::SharedPtr srv_;
};

}
#endif
