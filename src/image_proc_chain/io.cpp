#include "image_proc_chain/io.hpp"

#include <iostream>

using std::placeholders::_1;

namespace image_proc_chain {

IO::IO(const std::string& node_name) : Node(node_name) {
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      "~/image_in", 1, std::bind(&IO::Process, this, _1));
  pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_out", 1);
  RCLCPP_INFO(get_logger(), "%s has initialized", node_name.c_str());
}

void IO::Process(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::cout << "hello" << std::endl;
  pub_->publish(*msg);
}

}
