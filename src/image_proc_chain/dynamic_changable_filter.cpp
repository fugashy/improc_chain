#include "image_proc_chain/dynamic_changable_filter.hpp"

#include "image_proc_chain/factory.hpp"

namespace image_proc_chain {

DynamicChangableFilter::DynamicChangableFilter(ros::NodeHandle& nh)
  : nh_(nh) {
  server_ = nh.advertiseService(
      "change_filter", &DynamicChangableFilter::ChangeFilterType, this);
  filter_ = Factory::Create(nh_);
}

void DynamicChangableFilter::Through(const cv::Mat& in, cv::Mat& out) {
  filter_->Through(in, out);
}

bool DynamicChangableFilter::ChangeFilterType(
    image_proc_chain::FilterType::Request& req,
    image_proc_chain::FilterType::Response& res) {
  try {
    const std::string param_name = nh_.getNamespace() + "/type";
    ros::param::set(param_name, req.filter_type);
    filter_ = Factory::Create(nh_);
  } catch (const std::exception& e) {
    ROS_WARN("%s", e.what());
    return false;
  }

  return true;
}

}
