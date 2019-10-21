// Copyright 2019 fugashy
#include <memory>

#include "image_proc_chain/chain_processor.hpp"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("chain_processor");
  auto cp = std::make_shared<image_proc_chain::ChainProcessor>(node);

  cp->get_executor()->spin();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
