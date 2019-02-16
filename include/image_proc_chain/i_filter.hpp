#ifndef IMAGE_PROC_CHAIN_I_FILTER_HPP_
#define IMAGE_PROC_CHAIN_I_FILTER_HPP_
#include <memory>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>

namespace image_proc_chain {

class Interface {
 public:
  typedef std::shared_ptr<Interface> Ptr;
  virtual void Through(const cv::Mat& in, cv::Mat& out) = 0;
};
}
#endif

