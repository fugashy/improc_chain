#ifndef IMAGE_PROC_CHAIN_RECONFIGUAGLE_FILTER_HPP_
#define IMAGE_PROC_CHAIN_RECONFIGUAGLE_FILTER_HPP_
#include "image_proc_chain/i_filter.hpp"

#include <mutex>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <image_proc_chain/BilateralConfig.h>
#include <image_proc_chain/CannyEdgeConfig.h>
#include <image_proc_chain/GammaCorrectionConfig.h>
#include <image_proc_chain/GaussianConfig.h>

using image_proc_chain::BilateralConfig;
using image_proc_chain::CannyEdgeConfig;
using image_proc_chain::GammaCorrectionConfig;
using image_proc_chain::GaussianConfig;

namespace image_proc_chain {

class Bilateral : public Interface {
 public:
  explicit Bilateral(ros::NodeHandle& nh);
  virtual void Through(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(BilateralConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<BilateralConfig>> server_;
  BilateralConfig config_;
  std::mutex mutex_;
};

class CannyEdge : public Interface {
 public:
  explicit CannyEdge(ros::NodeHandle& nh);
  virtual void Through(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(CannyEdgeConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<CannyEdgeConfig>> server_;
  CannyEdgeConfig config_;
  std::mutex mutex_;
};

class GammaCorrection : public Interface {
 public:
  explicit GammaCorrection(ros::NodeHandle& nh);
  virtual void Through(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(GammaCorrectionConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<GammaCorrectionConfig>> server_;
  GammaCorrectionConfig config_;
  std::mutex mutex_;
  cv::Mat lut_;
};

class Gaussian : public Interface {
 public:
  explicit Gaussian(ros::NodeHandle& nh);
  virtual void Through(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(GaussianConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<GaussianConfig>> server_;
  GaussianConfig config_;
  std::mutex mutex_;
};

class PassThrough : public Interface {
 public:
  explicit PassThrough(ros::NodeHandle& nh);
  virtual void Through(const cv::Mat& in, cv::Mat& out);
};

}
#endif
