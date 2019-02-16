#include "image_proc_chain/factory.hpp"

#include <ros/ros.h>

#include "image_proc_chain/reconfiguable_filters.hpp"


namespace image_proc_chain {

Interface::Ptr Factory::Create(ros::NodeHandle& nh) {
  std::string type;
  nh.param<std::string>("type", type, "pass_through");

  Interface::Ptr ptr;
  if (type == "gaussian") {
    ptr.reset(new Gaussian(nh));
  } else if (type == "bilateral") {
    ptr.reset(new Bilateral(nh));
  } else if (type == "canny_edge") {
    ptr.reset(new CannyEdge(nh));
  } else if (type == "gamma_correction") {
    ptr.reset(new GammaCorrection(nh));
  } else if (type == "pass_through") {
    ptr.reset(new PassThrough(nh));
  } else {
    throw std::runtime_error("Invalid filter type.");
  }

  return ptr;
}

}
