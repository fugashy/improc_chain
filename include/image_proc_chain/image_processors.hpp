// Copyright 2019 fugashy
#ifndef IMAGE_PROC_CHAIN__IMAGE_PROCESSORS_HPP_
#define IMAGE_PROC_CHAIN__IMAGE_PROCESSORS_HPP_
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace image_proc_chain {
namespace image_processors {

class Base {
 public:
  /**
   * @brief Alias to std::shared_ptr
   */
  using SharedPtr = std::shared_ptr<Base>;

  /**
   * @brief Keep pointer to node
   *
   * @param[in] node Pointer to node
   */
  explicit Base(rclcpp::Node* node);

  /**
   * @brief Default destructor
   */
  virtual ~Base() = default;

  /**
   * @brief Interface to image processing
   *
   * This is pure virtual function
   *
   * @param[in] image_in Input image
   *
   * @return Processed image
   */
  virtual cv::Mat Process(const cv::Mat& image_in) = 0;

 protected:
  /**
   * @brief Pointer to node
   *
   * Used to get/set parameter, for logging
   */
  rclcpp::Node* node_;

  /**
   * @brief Pointer to a handlaer that manage parameter callbacks
   */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;
  rcl_interfaces::msg::ParameterDescriptor param_desc_;
};

class GaussianSpacial : public Base {
 public:
  /**
   * @brief Name of this processor
   */
  static const char ProcName[];

  /**
   * @brief Bind parameter callback and declare parameters
   *
   * @param[in] node Pointer to node
   */
  explicit GaussianSpacial(rclcpp::Node* node);

  /**
   * @brief Declare parameters
   */
  virtual ~GaussianSpacial();

  /**
   * @brief Gaussian spacial filtering
   *
   * @param[in] image_in Input image
   *
   * @return Image processed by gaussian spacial filter
   */
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  /**
   * @brief Reconfigure processor parameter
   *
   * @param[in] params Node parameters
   *
   * @return Success/Failure
   */
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  /**
   * @brief Kernel size of gaussian spacial filter
   */
  cv::Size kernel_;

  /**
   * @brief Sigma of gaussian
   */
  cv::Vec2d sigma_;

  /**
   * @brief Reiteration count of processing
   *
   * In case of gaussian spacial filter, computing cost less than making spacial parameters large.
   */
  uint32_t iteration_count_;
};

class Diration : public Base {
 public:
  /**
   * @brief Name of this processor
   */
  static const char ProcName[];

  /**
   * @brief Bind parameter callback and declare parameters
   *
   * @param[in] node Pointer to node
   */
  explicit Diration(rclcpp::Node* node);

  /**
   * @brief Declare parameters
   */
  virtual ~Diration();

  /**
   * @brief Diration
   *
   * @param[in] image_in Input image
   *
   * @return Image processed
   */
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  /**
   * @brief Reconfigure processor parameter
   *
   * @param[in] params Node parameters
   *
   * @return Success/Failure
   */
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  /**
   * @brief Kernel size and shape
   */
  cv::Mat kernel_;

  /**
   * @brief Reiteration count of processing
   */
  uint32_t iteration_count_;
};


class Erosion : public Base {
 public:
  /**
   * @brief Name of this processor
   */
  static const char ProcName[];

  /**
   * @brief Bind parameter callback and declare parameters
   *
   * @param[in] node Pointer to node
   */
  explicit Erosion(rclcpp::Node* node);

  /**
   * @brief Declare parameters
   */
  virtual ~Erosion();

  /**
   * @brief Erosion
   *
   * @param[in] image_in Input image
   *
   * @return Image processed
   */
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  /**
   * @brief Reconfigure processor parameter
   *
   * @param[in] params Node parameters
   *
   * @return Success/Failure
   */
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  /**
   * @brief Kernel size of gaussian spacial filter
   */
  cv::Mat kernel_;

  /**
   * @brief Reiteration count of processing
   */
  uint32_t iteration_count_;
};


class CannyEdge : public Base {
 public:
  /**
   * @brief Name of this processor
   */
  static const char ProcName[];

  /**
   * @brief Bind parameter callback and declare parameters
   *
   * @param[in] node Pointer to node
   */
  explicit CannyEdge(rclcpp::Node* node);

  /**
   * @brief Declare parameters
   */
  virtual ~CannyEdge();

  /**
   * @brief Canny edge detection
   *
   * @param[in] image_in Input image
   *
   * @return Image processed
   */
  virtual cv::Mat Process(const cv::Mat& image_in);

 private:
  /**
   * @brief Reconfigure processor parameter
   *
   * @param[in] params Node parameters
   *
   * @return Success/Failure
   */
  rcl_interfaces::msg::SetParametersResult ChangeParameters(
      const std::vector<rclcpp::Parameter>& params);

  /**
   * @brief higher threshold for hysteresis procedure
   */
  uint32_t val_max_;

  /**
   * @brief lower threshold for hysteresis procedure
   */
  uint32_t val_min_;

  /**
   * @brief Aperture size for Sobel operator
   */
  uint32_t sobel_aperture_;
};

/**
 * @brief Verify specified name of type is available(or implemented)
 *
 * @param[in] type_name Name of type for verification
 *
 * @return Available or not
 */
bool IsAvailable(const std::string& type_name);

/**
 * @brief Factory function to create inherited classes of Base
 *
 * @param[in] node Pointer to node
 *
 * @return Pointer to base class
 */
Base::SharedPtr Create(rclcpp::Node* node);

}  // namespace image_processors
}  // namespace image_proc_chain

#endif  // IMAGE_PROC_CHAIN__IMAGE_PROCESSORS_HPP_
