#include "image_proc_chain/reconfiguable_filters.hpp"

#include <opencv2/imgproc.hpp>

namespace image_proc_chain {

CannyEdge::CannyEdge(ros::NodeHandle& nh) {
  server_.reset(new dynamic_reconfigure::Server<CannyEdgeConfig>(
        ros::NodeHandle(nh, "canny_edge")));
  dynamic_reconfigure::Server<CannyEdgeConfig>::CallbackType f = boost::bind(
      &CannyEdge::ReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void CannyEdge::Through(const cv::Mat& in, cv::Mat& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  cv::Mat tmp = in.clone();
  if (in.type() != CV_8UC1) {
    cv::cvtColor(in, tmp, CV_RGB2GRAY);
  }
  cv::Canny(
      tmp, out, config_.val_min, config_.val_max, config_.sobel_aperture);
}

void CannyEdge::ReconfigureCallback(CannyEdgeConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (config.val_max == config.val_min) {
    ROS_WARN("Min value and max value have same value each other.");
    return;
  } else if (config.sobel_aperture < 3 || 7 < config.sobel_aperture ||
             config.sobel_aperture % 2 != 1) {
    ROS_WARN("Aperture size should be odd between 3 and 7");
    return;
  }
  config_ = config;
}

}
