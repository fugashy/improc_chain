#include "image_proc_chain/reconfigurable_processors.hpp"

#include <opencv2/imgproc.hpp>

namespace image_proc_chain {

Gaussian::Gaussian(ros::NodeHandle& nh) {
  server_.reset(new dynamic_reconfigure::Server<GaussianConfig>(
        ros::NodeHandle(nh, "gaussian")));
  dynamic_reconfigure::Server<GaussianConfig>::CallbackType f = boost::bind(
      &Gaussian::ReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void Gaussian::Work(const cv::Mat& in, cv::Mat& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.enable) {
    out = in;
    return;
  }
  const cv::Size kernel(config_.kernel_x, config_.kernel_y);
  cv::Mat tmp = in;
  for (int i = 0, iend = config_.iteration_count; i < iend; ++i) {
    cv::GaussianBlur(
        tmp, out, kernel, config_.sigma_x, config_.sigma_y, config_.border_type);
    tmp = out.clone();
  }
}

void Gaussian::ReconfigureCallback(GaussianConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (config.kernel_x % 2 != 1 or config.kernel_x < 1 or
      config.kernel_y % 2 != 1 or config.kernel_y < 1) {
    ROS_WARN("Kernel size should be > 1 and odd num.");
  }
  config_ = config;
}

}
