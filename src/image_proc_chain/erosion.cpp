#include "image_proc_chain/reconfigurable_processors.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace image_proc_chain {

Erosion::Erosion(ros::NodeHandle& nh) {
  server_.reset(new dynamic_reconfigure::Server<ErosionConfig>(
        ros::NodeHandle(nh, "erosion")));
  dynamic_reconfigure::Server<ErosionConfig>::CallbackType f = boost::bind(
      &Erosion::ReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void Erosion::Work(const cv::Mat& in, cv::Mat& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.enable) {
    out = in;
    return;
  }

  cv::erode(in, out, kernel_, cv::Point(-1, -1), config_.iteration_count);
}

void Erosion::ReconfigureCallback(ErosionConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;

  kernel_ = cv::Mat::ones(config_.kernel_size, config_.kernel_size, CV_8UC1);
}

}
