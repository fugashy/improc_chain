#ifndef IMAGE_PROC_CHAIN_NODELET_HPP_
#define IMAGE_PROC_CHAIN_NODELET_HPP_
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>

#include "image_proc_chain/dynamic_changable_filter.hpp"

namespace image_proc_chain {

class Nodelet : public nodelet::Nodelet {
 public:
  virtual void onInit();

 private:
  void Callback(const sensor_msgs::ImageConstPtr& msg);

  DynamicChangableFilter::Ptr filter_;

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
};

}

PLUGINLIB_EXPORT_CLASS(image_proc_chain::Nodelet, nodelet::Nodelet)
#endif
