#include "image_proc_chain/reconfigurable_processors.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace image_proc_chain {

Dilation::Dilation(ros::NodeHandle& nh) {
  server_.reset(new dynamic_reconfigure::Server<DilationConfig>(
        ros::NodeHandle(nh, "dilation")));
  dynamic_reconfigure::Server<DilationConfig>::CallbackType f = boost::bind(
      &Dilation::ReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void Dilation::Work(const cv::Mat& in, cv::Mat& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.enable) {
    out = in;
    return;
  }

  cv::dilate(in, out, kernel_, cv::Point(-1, -1), config_.iteration_count);
}

void Dilation::ReconfigureCallback(DilationConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;

  kernel_ = cv::Mat::ones(config_.kernel_size, config_.kernel_size, CV_8UC1);
}

}
