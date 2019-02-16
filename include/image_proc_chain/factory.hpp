#ifndef IMAGE_PROC_CHAIN_FACTORY_HPP_
#define IMAGE_PROC_CHAIN_FACTORY_HPP_
#include "image_proc_chain/i_filter.hpp"

namespace image_proc_chain {

class Factory {
 public:
  static Interface::Ptr Create(ros::NodeHandle& nh);
};

}
#endif
