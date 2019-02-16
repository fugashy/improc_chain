#ifndef IMAGE_PROC_CHAIN_CHAIN_HPP_
#define IMAGE_PROC_CHAIN_CHAIN_HPP_
#include <memory>

#include <image_proc_chain/ChangeChainNum.h>
#include <nodelet/loader.h>
#include <ros/ros.h>

namespace image_proc_chain {

class Chain {
 public:
  typedef std::shared_ptr<Chain> Ptr;
  explicit Chain(ros::NodeHandle& nh);

 private:
  bool ChangeChainNum(image_proc_chain::ChangeChainNum::Request& req,
                      image_proc_chain::ChangeChainNum::Response& res);

  std::shared_ptr<nodelet::Loader> manager_;
  ros::ServiceServer server_;
};

}
#endif
