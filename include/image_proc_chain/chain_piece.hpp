// Copyright 2019 fugashy
#ifndef IMAGE_PROC_CHAIN__CHAIN_PIECE_HPP_
#define IMAGE_PROC_CHAIN__CHAIN_PIECE_HPP_
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_proc_chain/switchable_image_processor.hpp"

namespace image_proc_chain {

class ChainPiece {
 public:
  /**
   * Alias to std::shared_ptr
   **/
  using SharedPtr = std::shared_ptr<ChainPiece>;

  /**
   * @brief Build pub/sub and initialize SwitchableImageProcessor
   *
   * @param[in] node Shared pointer to node
   * @param[in] in_topic_name Name of image topic to input
   **/
  explicit ChainPiece(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& in_topic_name = "/camera/image_raw");

 private:

  /**
   * @brief Subscribe image, then publish processed image
   *
   * 1. Convert image topic to cv::Mat
   * 2. Image processing
   * 3. Convert processed image to image topic
   * 4. Publish
   *
   * @param msg Image topic
   */
  void Process(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Pointer to node
   *
   * Used for logging
   */
  std::shared_ptr<rclcpp::Node> node_;

  /**
   * @brief Pointer to image processor that can be switch the type of processor
   */
  SwitchableImageProcessor::SharedPtr image_processor_;

  /**
   * @brief Pointer to image subscriber
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  /**
   * @brief Pointer to image publisher
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__CHAIN_PIECE_HPP_
