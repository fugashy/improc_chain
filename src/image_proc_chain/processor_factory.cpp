#include "image_proc_chain/processor_factory.hpp"

#include <ros/ros.h>

#include "image_proc_chain/reconfigurable_processors.hpp"


namespace image_proc_chain {

IProcessor::Ptr ProcessorFactory::Create(ros::NodeHandle& nh) {
  std::string type;
  nh.param<std::string>("type", type, "gaussian");

  IProcessor::Ptr ptr;
  if (type == "gaussian") {
    ptr.reset(new Gaussian(nh));
  } else if (type == "bilateral") {
    ptr.reset(new Bilateral(nh));
  } else if (type == "canny_edge") {
    ptr.reset(new CannyEdge(nh));
  } else if (type == "gamma_correction") {
    ptr.reset(new GammaCorrection(nh));
  } else {
    throw std::runtime_error("Invalid filter type.");
  }

  ROS_INFO("Create %s processor.", type.c_str());
  return ptr;
}

}
