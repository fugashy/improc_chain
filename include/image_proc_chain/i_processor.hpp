#ifndef IMAGE_PROC_CHAIN_I_PROCESSOR_HPP_
#define IMAGE_PROC_CHAIN_I_PROCESSOR_HPP_
#include <memory>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>

namespace image_proc_chain {

class IProcessor {
 public:
  typedef std::shared_ptr<IProcessor> Ptr;
  virtual void Work(const cv::Mat& in, cv::Mat& out) = 0;
};
}
#endif

