#include "image_proc_chain/chain.hpp"

#include <ros/ros.h>

using image_proc_chain::Chain;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "filter_chain_node");
  ros::NodeHandle private_nh("~");
  Chain::Ptr chain;

  try {
    chain.reset(new Chain(private_nh));
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return -1;
  }
}
