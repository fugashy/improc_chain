#include "image_proc_chain/nodelet.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "image_proc_chain/factory.hpp"

namespace image_proc_chain {

void Nodelet::onInit() {
  // Publisher
  ros::NodeHandle pnh = getPrivateNodeHandle();
  image_transport::ImageTransport itp(pnh);
  pub_ = itp.advertise("image_filtered", 1);

  // Subscriber
  ros::NodeHandle nh = getNodeHandle();
  image_transport::ImageTransport its(nh);
  sub_ = its.subscribe("camera/image_raw", 1, &Nodelet::Callback, this);

  filter_.reset(new DynamicChangableFilter(pnh));
}

void Nodelet::Callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat out;
  filter_->Through(cv_ptr->image, out);

  std::string encoding;
  if (out.type() == CV_8UC1) {
    encoding = "mono8";
  } else if (out.type() == CV_8UC3) {
    encoding = "bgr8";
  } else {
    NODELET_WARN_THROTTLE(3.0, "Type of filtered image is invalid");
    return;
  }

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(
      msg->header, encoding, out).toImageMsg();
  pub_.publish(out_msg);
}

}
