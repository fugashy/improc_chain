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
  using SharedPtr = std::shared_ptr<ChainPiece>;

  explicit ChainPiece(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& in_topic_name = "/camera/image_raw");

 private:
  void Process(const sensor_msgs::msg::Image::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  SwitchableImageProcessor::SharedPtr image_processor_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__CHAIN_PIECE_HPP_
