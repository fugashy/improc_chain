#include "image_proc_chain/chain.hpp"

#include <sstream>
#include <iomanip>

namespace {
const char* const kTopicOfInput = "/camera/image_raw";
const char* const kTopicOfFiltered = "/image_filtered";
const char* const kNameOfNodelet = "image_proc_chain/Nodelet";
const int kMinChainNum = 1;
const int kMaxChainNum = 8;
}

namespace image_proc_chain {

Chain::Chain(ros::NodeHandle& nh) {
  server_ = nh.advertiseService(
      "change_chain_num", &Chain::ChangeChainNum, this);
  manager_.reset(new nodelet::Loader());

  int chain_num;
  nh.param<int>("default_chain_num", chain_num, 3);
  if (chain_num < kMinChainNum || kMaxChainNum < chain_num) {
    throw std::runtime_error("Number of chain should be in range of 1 to 8");
  }
  std::string input_topic;
  nh.param<std::string>("input_topic", input_topic, kTopicOfInput);
  if (input_topic == std::string()) {
    throw std::runtime_error("Name of input topic is empty.");
  }

  for (int i = 0; i < chain_num; ++i) {
    // Generate remappings
    const std::string target_to_remap =
        ros::this_node::getName() + std::string("/") + kTopicOfInput;
    nodelet::M_string remap;
    if (i == 0) {
      remap[target_to_remap] = input_topic;
    } else {
      const std::string last_nodelet_name = manager_->listLoadedNodelets().back();
      remap[target_to_remap] = last_nodelet_name + kTopicOfFiltered;
    }

    // Generate nodelet name
    std::ostringstream sout;
    sout << "filter_" << std::setfill('0') << std::setw(3) << i;
    const std::string nodelet_name =
        ros::this_node::getName() + std::string("/") + sout.str();

    // Load nodelet
    if (!manager_->load(nodelet_name, kNameOfNodelet, remap, nodelet::V_string())) {
      const std::string error_msg = std::string("Failed to load ") + nodelet_name;
      throw std::runtime_error(error_msg);
    }
  }

  ROS_INFO("Ready to filtering");
}

bool Chain::ChangeChainNum(image_proc_chain::ChangeChainNum::Request& req,
                           image_proc_chain::ChangeChainNum::Response& res) {
  // When loading, it may die suddenly.
  ROS_WARN("This service is unstable...");
  if (req.num < kMinChainNum || kMaxChainNum < req.num) {
    ROS_WARN("Number of chain should be in range of 1 to 8");
    return false;
  }
  const int current_chain_num = manager_->listLoadedNodelets().size();
  const int diff = req.num - current_chain_num;
  if (diff == 0) {
    ROS_WARN("Requested chain num is same as current chain num. We ignore this request.");
    return false;
  }

  if (diff > 0) {
    // Load nodelets
    for (int i = current_chain_num, iend = req.num; i < iend; ++i) {
      // Generate remappings
      nodelet::M_string remap;
      const std::string last_nodelet_name = manager_->listLoadedNodelets().back();
      const std::string target_to_remap =
          ros::this_node::getName() + std::string("/") + kTopicOfInput;
      remap[target_to_remap] = last_nodelet_name + kTopicOfFiltered;

      // Generate nodelet name
      std::ostringstream sout;
      sout << "filter_" << std::setfill('0') << std::setw(3) << i;
      const std::string nodelet_name =
          ros::this_node::getName() + std::string("/") + sout.str();

      // Load nodelet
      if (!manager_->load(nodelet_name, kNameOfNodelet, remap, nodelet::V_string())) {
        ROS_WARN("Failed to load %s", nodelet_name.c_str());
        return false;
      }
      ROS_INFO("Succeed to load %s", nodelet_name.c_str());
    }
  } else {
    // Unload unnecessary nodelets
    const auto nodelet_list = manager_->listLoadedNodelets();
    for (int i = req.num, iend = current_chain_num; i < iend; ++i) {
      manager_->unload(nodelet_list[i]);
      ROS_INFO("Succeed to unload %s", nodelet_list[i].c_str());
    }
  }

  return true;
}

}
