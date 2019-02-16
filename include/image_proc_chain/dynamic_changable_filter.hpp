#ifndef IMAGE_PROC_CHAIN_DYNAMIC_CHANGABLE_FILTER_HPP_
#define IMAGE_PROC_CHAIN_DYNAMIC_CHANGABLE_FILTER_HPP_
#include <ros/ros.h>
#include <image_proc_chain/FilterType.h>

#include "image_proc_chain/i_filter.hpp"

namespace image_proc_chain {

class DynamicChangableFilter {
 public:
  typedef std::shared_ptr<DynamicChangableFilter> Ptr;
  explicit DynamicChangableFilter(ros::NodeHandle& nh);
  void Through(const cv::Mat& in, cv::Mat& out);

 private:
  bool ChangeFilterType(image_proc_chain::FilterType::Request& req,
                        image_proc_chain::FilterType::Response& res);

  ros::NodeHandle nh_;
  Interface::Ptr filter_;
  ros::ServiceServer server_;
};
}
#endif
