#include "image_proc_chain/io.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>


using std::placeholders::_1;

namespace image_proc_chain {

IO::IO(const std::string& node_name) : Node(node_name) {
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      "~/image_in", 1, std::bind(&IO::Process, this, _1));
  pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_out", 1);
  RCLCPP_INFO(get_logger(), "%s has initialized", node_name.c_str());
}

void IO::Process(const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "subscription event has come to");

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "%d %d", cv_ptr->image.rows, cv_ptr->image.cols);

  pub_->publish(*msg);
}

}
