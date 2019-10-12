#include "image_proc_chain/chain_piece.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>


using std::placeholders::_1;

namespace image_proc_chain {

ChainPiece::ChainPiece(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {
  sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "~/image_in", 1, std::bind(&ChainPiece::Process, this, _1));
  pub_ = node_->create_publisher<sensor_msgs::msg::Image>("~/image_out", 1);
  RCLCPP_INFO(node_->get_logger(), "%s has initialized", node_->get_name());
}

void ChainPiece::Process(const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(node_->get_logger(), "subscription event has come to");

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "%d %d", cv_ptr->image.rows, cv_ptr->image.cols);

  pub_->publish(*msg);
}

}
