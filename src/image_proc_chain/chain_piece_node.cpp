// Copyright 2019 fugashy
#include <memory>

#include "image_proc_chain/chain_piece.hpp"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("chain_piece");
  auto cp = std::make_shared<image_proc_chain::ChainPiece>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
