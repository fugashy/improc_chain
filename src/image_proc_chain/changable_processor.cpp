#include "image_proc_chain/changable_processor.hpp"

#include "image_proc_chain/processor_factory.hpp"

namespace image_proc_chain {

ChangableProcessor::ChangableProcessor(ros::NodeHandle& nh)
  : nh_(nh) {
  server_ = nh.advertiseService(
      "change_filter", &ChangableProcessor::ChangeProcessorType, this);
  processor_ = ProcessorFactory::Create(nh_);
}

void ChangableProcessor::Work(const cv::Mat& in, cv::Mat& out) {
  processor_->Work(in, out);
}

bool ChangableProcessor::ChangeProcessorType(
    image_proc_chain::ChangeProcessorType::Request& req,
    image_proc_chain::ChangeProcessorType::Response& res) {
  try {
    const std::string param_name = nh_.getNamespace() + "/type";
    ros::param::set(param_name, req.type);
    processor_ = ProcessorFactory::Create(nh_);
  } catch (const std::exception& e) {
    ROS_WARN("%s", e.what());
    return false;
  }

  return true;
}

}
