#include "image_proc_chain/reconfiguable_filters.hpp"

#include <opencv2/imgproc.hpp>

namespace image_proc_chain {

Bilateral::Bilateral(ros::NodeHandle& nh) {
  server_.reset(new dynamic_reconfigure::Server<BilateralConfig>(
        ros::NodeHandle(nh, "bilateral")));
  dynamic_reconfigure::Server<BilateralConfig>::CallbackType f = boost::bind(
      &Bilateral::ReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void Bilateral::Through(const cv::Mat& in, cv::Mat& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  cv::Mat tmp = in;
  for (int i = 0, iend = config_.iteration_count; i < iend; ++i) {
    cv::bilateralFilter(
        tmp, out, config_.sigma_color, config_.sigma_space, config_.border_type);
    tmp = out.clone();
  }
}

void Bilateral::ReconfigureCallback(BilateralConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

}
