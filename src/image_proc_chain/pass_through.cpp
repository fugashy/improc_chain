#include "image_proc_chain/reconfiguable_filters.hpp"


namespace image_proc_chain {

PassThrough::PassThrough(ros::NodeHandle& nh) {}

void PassThrough::Through(const cv::Mat& in, cv::Mat& out) {
  out = in;
}

}
