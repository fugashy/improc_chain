#ifndef IMAGE_PROC_CHAIN_RECONFIGURABLE_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN_RECONFIGURABLE_PROCESSOR_HPP_
#include "image_proc_chain/i_processor.hpp"

#include <mutex>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <image_proc_chain/BilateralConfig.h>
#include <image_proc_chain/CannyEdgeConfig.h>
#include <image_proc_chain/DilationConfig.h>
#include <image_proc_chain/ErosionConfig.h>
#include <image_proc_chain/GammaCorrectionConfig.h>
#include <image_proc_chain/GaussianConfig.h>

namespace image_proc_chain {

class Bilateral : public IProcessor {
 public:
  explicit Bilateral(ros::NodeHandle& nh);
  virtual void Work(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(BilateralConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<BilateralConfig>> server_;
  BilateralConfig config_;
  std::mutex mutex_;
};

class CannyEdge : public IProcessor {
 public:
  explicit CannyEdge(ros::NodeHandle& nh);
  virtual void Work(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(CannyEdgeConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<CannyEdgeConfig>> server_;
  CannyEdgeConfig config_;
  std::mutex mutex_;
};

class Dilation : public IProcessor {
 public:
  explicit Dilation(ros::NodeHandle& nh);
  virtual void Work(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(DilationConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<DilationConfig>> server_;
  DilationConfig config_;
  std::mutex mutex_;

  cv::Mat kernel_;
};

class Erosion : public IProcessor {
 public:
  explicit Erosion(ros::NodeHandle& nh);
  virtual void Work(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(ErosionConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<ErosionConfig>> server_;
  ErosionConfig config_;
  std::mutex mutex_;

  cv::Mat kernel_;
};

class GammaCorrection : public IProcessor {
 public:
  explicit GammaCorrection(ros::NodeHandle& nh);
  virtual void Work(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(GammaCorrectionConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<GammaCorrectionConfig>> server_;
  GammaCorrectionConfig config_;
  std::mutex mutex_;
  cv::Mat lut_;
};

class Gaussian : public IProcessor {
 public:
  explicit Gaussian(ros::NodeHandle& nh);
  virtual void Work(const cv::Mat& in, cv::Mat& out);

 private:
  void ReconfigureCallback(GaussianConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<GaussianConfig>> server_;
  GaussianConfig config_;
  std::mutex mutex_;
};

}
#endif
