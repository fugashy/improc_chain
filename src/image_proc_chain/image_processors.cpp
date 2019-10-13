#include "image_proc_chain/image_processors.hpp"

#include <chrono>
#include <ratio>

#include <opencv2/imgproc.hpp>

using std::placeholders::_1;

namespace image_proc_chain {
namespace image_processors {


Base::Base(std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

GaussianSpacial::GaussianSpacial(std::shared_ptr<rclcpp::Node>& node)
    : Base(node),
      kernel_(cv::Size(21, 21)),
      sigma_(cv::Vec2d(1.0, 1.0)),
      iteration_count_(1)  {
  node->set_on_parameters_set_callback(std::bind(&GaussianSpacial::ChangeParameters, this, _1));
  node->declare_parameter("kernel_x", 21);
  node->declare_parameter("kernel_y", 21);
  node->declare_parameter("sigma_x", 1.0);
  node->declare_parameter("sigma_y", 1.0);
  node->declare_parameter("iteration_count", 1);
}

GaussianSpacial::~GaussianSpacial() {
  node_->undeclare_parameter("kernel_x");
  node_->undeclare_parameter("kernel_y");
  node_->undeclare_parameter("sigma_x");
  node_->undeclare_parameter("sigma_y");
  node_->undeclare_parameter("iteration_count");
}

cv::Mat GaussianSpacial::Process(const cv::Mat& image_in) {
  cv::Mat tmp = image_in;
  cv::Mat image_out;
  for (int i = 0, iend = iteration_count_; i < iend; ++i) {
    cv::GaussianBlur(
        tmp, image_out, kernel_, sigma_[0], sigma_[1]);
    tmp = image_out.clone();
  }

  return image_out;
}

rcl_interfaces::msg::SetParametersResult GaussianSpacial::ChangeParameters(
    const std::vector<rclcpp::Parameter>& params) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (auto param : params) {
    if (param.get_name() == "kernel_x") {
      const int width = param.as_int();
      if (width % 2 != 1 or width < 1) {
        result.successful = false;
      } else {
        kernel_.width = param.as_int();
      }
    }
    if (param.get_name() == "kernel_y") {
      const int height = param.as_int();
      if (height % 2 != 1 or height < 1) {
        result.successful = false;
      } else {
        kernel_.height = param.as_int();
      }
    }
    if (param.get_name() == "sigma_x") {
      sigma_[0] = param.as_double();
    }
    if (param.get_name() == "sigma_y") {
      sigma_[1] = param.as_double();
    }
    if (param.get_name() == "iteration_count") {
      iteration_count_ = param.as_int();
    }
  }

  return result;
}

Diration::Diration(std::shared_ptr<rclcpp::Node>& node)
    : Base(node),
      kernel_(cv::Mat::ones(3, 3, CV_8UC1)),
      iteration_count_(1) {
  node->set_on_parameters_set_callback(std::bind(&Diration::ChangeParameters, this, _1));
  node->declare_parameter("kernel_size", 3);
  node->declare_parameter("iteration_count", 1);
}

Diration::~Diration() {
  node_->undeclare_parameter("kernel_size");
  node_->undeclare_parameter("iteration_count");
}

cv::Mat Diration::Process(const cv::Mat& image_in) {
  cv::Mat image_out;
  cv::dilate(image_in, image_out, kernel_, cv::Point(-1, -1), iteration_count_);

  return image_out;
}

rcl_interfaces::msg::SetParametersResult Diration::ChangeParameters(
    const std::vector<rclcpp::Parameter>& params) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (auto param : params) {
    if (param.get_name() == "kernel_size") {
      const int size = param.as_int();
      if (size < 1) {
        result.successful = false;
      } else {
        kernel_ = cv::Mat::ones(size, size, CV_8UC1);
      }
    }
    if (param.get_name() == "iteration_count") {
      const int count = param.as_int();
      if (count < 1) {
        result.successful = false;
      } else {
        iteration_count_ = count;
      }
    }
  }

  return result;
}

Base::SharedPtr Create(std::shared_ptr<rclcpp::Node> node) {
  RCLCPP_INFO(node->get_logger(), "trying to change param");
  std::string type_str;
  rclcpp::Parameter type;
  if (!node->get_parameter("type", type)) {
    RCLCPP_WARN(node->get_logger(), "Failed to get type, we use default type gaussian_spacial");
    type_str = "gaussian_spacial";
  } else {
    type_str = type.as_string();
  }
  
  RCLCPP_INFO(node->get_logger(), "done");

  Base::SharedPtr ptr;
  if (type_str == "gaussian_spacial") {
    ptr.reset(new GaussianSpacial(node));
  } else if (type_str == "diration") {
    RCLCPP_INFO(node->get_logger(), "Diration");
    ptr.reset(new Diration(node));
    RCLCPP_INFO(node->get_logger(), "Done");
  } else {
    const std::string err = type_str + " is not implemented";
    throw std::invalid_argument(err);
  }

  return ptr;
}

}
}
