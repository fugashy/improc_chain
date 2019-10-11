#ifndef IMAGE_PROC_CHAIN_PROCESSOR_FACTORY_HPP_
#define IMAGE_PROC_CHAIN_PROCESSOR_FACTORY_HPP_
#include "image_proc_chain/i_processor.hpp"

namespace image_proc_chain {

class ProcessorFactory {
 public:
  static IProcessor::Ptr Create(ros::NodeHandle& nh);
};

}
#endif
