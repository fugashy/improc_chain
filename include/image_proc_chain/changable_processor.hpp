#ifndef IMAGE_PROC_CHAIN_CHANGABLE_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN_CHANGABLE_PROCESSOR_HPP_
#include <ros/ros.h>
#include <image_proc_chain/ChangeProcessorType.h>

#include "image_proc_chain/i_processor.hpp"

namespace image_proc_chain {

class ChangableProcessor {
 public:
  typedef std::shared_ptr<ChangableProcessor> Ptr;
  explicit ChangableProcessor(ros::NodeHandle& nh);
  void Work(const cv::Mat& in, cv::Mat& out);

 private:
  bool ChangeProcessorType(image_proc_chain::ChangeProcessorType::Request& req,
                           image_proc_chain::ChangeProcessorType::Response& res);

  ros::NodeHandle nh_;
  IProcessor::Ptr processor_;
  ros::ServiceServer server_;
};
}
#endif
