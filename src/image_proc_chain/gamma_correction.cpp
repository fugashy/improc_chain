#include "image_proc_chain/reconfiguable_filters.hpp"

#include <opencv2/imgproc.hpp>

namespace {
double ConvertWithGC(double x, double gamma) {
  return 255.0 * std::pow(x / 255.0, 1.0 / gamma);
}
}

namespace image_proc_chain {

GammaCorrection::GammaCorrection(ros::NodeHandle& nh) {
  lut_ = cv::Mat(1, 256, CV_8U);

  server_.reset(new dynamic_reconfigure::Server<GammaCorrectionConfig>(
        ros::NodeHandle(nh, "gamma_correction")));
  dynamic_reconfigure::Server<GammaCorrectionConfig>::CallbackType f = boost::bind(
      &GammaCorrection::ReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void GammaCorrection::Through(const cv::Mat& in, cv::Mat& out) {
  std::lock_guard<std::mutex> lock(mutex_);
  cv::Mat tmp = in.clone();
  if (in.type() != CV_8UC1) {
    cv::cvtColor(in, tmp, CV_RGB2GRAY);
  }
  cv::LUT(tmp, lut_, out);
}

void GammaCorrection::ReconfigureCallback(GammaCorrectionConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (config.gamma <= 0.01) {
    ROS_WARN("Gamma should be larger than 0.01");
    return;
  }

  uint8_t* head = lut_.data;
  for (int i = 0; i < 256; ++i) {
    head[i] = ConvertWithGC(i, config.gamma);
  }
}

}
