// Copyright 2019 fugashy
#ifndef IMAGE_PROC_CHAIN__SWITCHABLE_IMAGE_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN__SWITCHABLE_IMAGE_PROCESSOR_HPP_
#include <memory>

#include <image_proc_chain_msgs/srv/switch_processor_type.hpp>

#include "image_proc_chain/image_processors.hpp"

namespace image_proc_chain {

class SwitchableImageProcessor {
 public:
  /**
   * @brief Alias to std::shared_ptr
   */
  using SharedPtr = std::shared_ptr<SwitchableImageProcessor>;

  /**
   * @brief Initialize image processor and advertise switching service
   *
   * @param[in] node Pointer to node
   */
  explicit SwitchableImageProcessor(rclcpp::Node* node);

  /**
   * @brief Process image by using image_processors::Base
   *
   * @param[in] image_in Input image
   *
   * @return Processed image
   */
  cv::Mat Process(const cv::Mat& image_in);

 private:
  /**
   * @brief Switch type of processor
   *
   * It is callback function of ros service
   *
   * @param[in] request_header Pointer to request header
   * @param[in] request Pointer to request
   * @param[in] response Pointer to response
   */
  void SwitchTypeOfProcessor(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<image_proc_chain_msgs::srv::SwitchProcessorType::Request> request,
      std::shared_ptr<image_proc_chain_msgs::srv::SwitchProcessorType::Response> response);

  /**
   * @brief Pointer to node
   */
  rclcpp::Node* node_;

  /**
   * @brief Pointer to image processor
   */
  image_processors::Base::SharedPtr image_processor_;

  /**
   * @brief Pointer to service server for switching processors
   */
  rclcpp::Service<image_proc_chain_msgs::srv::SwitchProcessorType>::SharedPtr srv_;
};

}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__SWITCHABLE_IMAGE_PROCESSOR_HPP_
