#include "image_proc_chain/chain.hpp"

#include <sstream>
#include <iomanip>

namespace {
const char* const kTopicOfInput = "/camera/image_raw";
const char* const kTopicOfFiltered = "/image_processed";
const char* const kNameOfNodelet = "image_proc_chain/Nodelet";
const int kMinChainNum = 1;
const int kRecommendedChainNum = 4;
const int kMaxChainNum = 8;

bool LoadLoop(int chain_num, const std::string& input_topic, std::shared_ptr<nodelet::Loader>& manager) {
  for (int i = 0; i < chain_num; ++i) {
    // Generate remappings
    const std::string target_to_remap =
        ros::this_node::getName() + kTopicOfInput;
    nodelet::M_string remap;
    if (i == 0) {
      remap[target_to_remap] = input_topic;
    } else {
      const std::string last_nodelet_name = manager->listLoadedNodelets().back();
      remap[target_to_remap] = last_nodelet_name + kTopicOfFiltered;
    }

    // Generate nodelet name
    std::ostringstream sout;
    sout << "/filter_" << std::setfill('0') << std::setw(3) << i;
    const std::string nodelet_name = ros::this_node::getName() + sout.str();

    // Load nodelet
    if (!manager->load(nodelet_name, kNameOfNodelet, remap, nodelet::V_string())) {
      ROS_ERROR("Failed to load %s", nodelet_name.c_str());
      return false;
    }
  }
  return true;
}
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
  if (chain_num > kRecommendedChainNum) {
    ROS_WARN("Large value (> %d) is unstable", kRecommendedChainNum);
  }
  nh.param<std::string>("input_topic", input_topic_, kTopicOfInput);
  if (input_topic_ == std::string()) {
    throw std::runtime_error("Name of input topic is empty.");
  }

  if (!LoadLoop(chain_num, input_topic_, manager_)) {
    throw std::runtime_error("Failed to initialize");
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
  if (req.num > kRecommendedChainNum) {
    ROS_WARN("Large value (> %d) is unstable", kRecommendedChainNum);
  }
  const int current_chain_num = manager_->listLoadedNodelets().size();
  if (current_chain_num == req.num) {
    ROS_WARN("Requested chain num is same as current chain num. We ignore this request.");
    return false;
  }

  manager_.reset(new nodelet::Loader());

  return LoadLoop(req.num, input_topic_, manager_);
}

}
