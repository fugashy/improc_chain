#ifndef IMAGE_PROC_CHAIN_IMAGE_PROCESSORS_HPP_
#define IMAGE_PROC_CHAIN_IMAGE_PROCESSORS_HPP_
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

namespace image_proc_chain {
namespace image_processors {

class Base {
 public:
  using SharedPtr = std::shared_ptr<Base>;
  Base(std::shared_ptr<rclcpp::Node>& node);
  virtual ~Base() = default;
  virtual cv::Mat Process(const cv::Mat& image_in) = 0;

 protected:
  std::shared_ptr<rclcpp::Node> node_;
};

class GaussianSpacial : public Base {
 public:
  explicit GaussianSpacial(std::shared_ptr<rclcpp::Node>& node);
  virtual ~GaussianSpacial();
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  cv::Size kernel_;
  cv::Vec2d sigma_;
  uint32_t iteration_count_;
};


}
}

#endif
