#include "image_proc_chain/io.hpp"

#include <iostream>

using std::placeholders::_1;

namespace image_proc_chain {

IO::IO(const std::string& node_name) : Node(node_name) {
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 1, std::bind(&IO::Process, this, _1));
}

void IO::Process(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::cout << "hello" << std::endl;
}

}
