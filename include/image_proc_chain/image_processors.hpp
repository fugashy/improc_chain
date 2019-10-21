// Copyright 2019 fugashy
#ifndef IMAGE_PROC_CHAIN__IMAGE_PROCESSORS_HPP_
#define IMAGE_PROC_CHAIN__IMAGE_PROCESSORS_HPP_
#include <memory>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace image_proc_chain {
namespace image_processors {

class Base {
 public:
  using SharedPtr = std::shared_ptr<Base>;
  explicit Base(std::shared_ptr<rclcpp::Node>& node);
  virtual ~Base() = default;

  virtual cv::Mat Process(const cv::Mat& image_in) = 0;

 protected:
  std::shared_ptr<rclcpp::Node> node_;
};

class GaussianSpacial : public Base {
 public:
  static const char ProcName[];
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

class Diration : public Base {
 public:
  static const char ProcName[];
  explicit Diration(std::shared_ptr<rclcpp::Node>& node);
  virtual ~Diration();
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  cv::Mat kernel_;
  uint32_t iteration_count_;
};


class Erosion : public Base {
 public:
  static const char ProcName[];
  explicit Erosion(rclcpp::Node::SharedPtr& node);
  virtual ~Erosion();
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  cv::Mat kernel_;
  uint32_t iteration_count_;
};


class CannyEdge : public Base {
 public:
  static const char ProcName[];
  explicit CannyEdge(rclcpp::Node::SharedPtr& node);
  virtual ~CannyEdge();
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  uint32_t val_max_;
  uint32_t val_min_;
  uint32_t sobel_aperture_;
};

bool IsAvailable(const std::string& type_name);
Base::SharedPtr Create(std::shared_ptr<rclcpp::Node> node);

}  // namespace image_processors
}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__IMAGE_PROCESSORS_HPP_
