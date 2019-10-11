#include "image_proc_chain/io.hpp"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<image_proc_chain::IO>("io");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
