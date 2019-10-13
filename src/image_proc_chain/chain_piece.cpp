#include "image_proc_chain/chain_piece.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>


using std::placeholders::_1;

namespace image_proc_chain {

ChainPiece::ChainPiece(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {
  image_processor_ = std::make_shared<SwitchableImageProcessor>(node);
  sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "~/image_in", 1, std::bind(&ChainPiece::Process, this, _1));
  pub_ = node_->create_publisher<sensor_msgs::msg::Image>("~/image_out", 1);
  RCLCPP_INFO(node_->get_logger(), "%s has initialized", node_->get_name());
}

void ChainPiece::Process(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat out = image_processor_->Process(cv_ptr->image);

  std::string encoding;
  if (out.type() == CV_8UC1) {
    encoding = "mono8";
  } else if (out.type() == CV_8UC3) {
    encoding = "bgr8";
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Type of filtered image is invalid");
    return;
  }

  sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(
      msg->header, encoding, out).toImageMsg();

  pub_->publish(*out_msg);
}

}
