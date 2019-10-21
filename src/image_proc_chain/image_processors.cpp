#include "image_proc_chain/image_processors.hpp"

#include <chrono>
#include <ratio>

#include <opencv2/imgproc.hpp>

using std::placeholders::_1;

namespace image_proc_chain {
namespace image_processors {


Base::Base(std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

const std::string GaussianSpacial::ProcName = "gaussian_spacial";

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

const std::string Diration::ProcName = "diration";

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

const std::string Erosion::ProcName = "erosion";

Erosion::Erosion(rclcpp::Node::SharedPtr& node)
    : Base(node),
      kernel_(cv::Mat::ones(3, 3, CV_8UC1)),
      iteration_count_(1) {
  node->set_on_parameters_set_callback(std::bind(&Erosion::ChangeParameters, this, _1));
  node->declare_parameter("kernel_size", 3);
  node->declare_parameter("iteration_count", 1);
}

Erosion::~Erosion() {
  node_->undeclare_parameter("kernel_size");
  node_->undeclare_parameter("iteration_count");
}

cv::Mat Erosion::Process(const cv::Mat& image_in) {
  cv::Mat image_out;
  cv::erode(image_in, image_out, kernel_, cv::Point(-1, -1), iteration_count_);

  return image_out;
}

rcl_interfaces::msg::SetParametersResult Erosion::ChangeParameters(
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

const std::string CannyEdge::ProcName = "canny_edge";

CannyEdge::CannyEdge(rclcpp::Node::SharedPtr& node)
    : Base(node),
      val_max_(100),
      val_min_(50),
      sobel_aperture_(3) {
  node->set_on_parameters_set_callback(std::bind(&CannyEdge::ChangeParameters, this, _1));
  node->declare_parameter("val_max", 100);
  node->declare_parameter("val_min", 50);
  node->declare_parameter("sobel_aperture", 3);
}

CannyEdge::~CannyEdge() {
  node_->undeclare_parameter("val_max");
  node_->undeclare_parameter("val_min");
  node_->undeclare_parameter("sobel_aperture");
}

cv::Mat CannyEdge::Process(const cv::Mat& image_in) {
  cv::Mat tmp = image_in.clone();
  if (image_in.type() != CV_8UC1) {
    cv::cvtColor(image_in, tmp, cv::COLOR_RGB2GRAY);
  }

  cv::Mat image_out;
  cv::Canny(tmp, image_out, val_min_, val_max_, sobel_aperture_);

  return image_out;
}

rcl_interfaces::msg::SetParametersResult CannyEdge::ChangeParameters(
    const std::vector<rclcpp::Parameter>& params) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  int val_max, val_min;
  for (auto param : params) {
    if (param.get_name() == "val_max") {
      val_max = param.as_int();
      if (val_max < 1) {
        RCLCPP_WARN(node_->get_logger(), "val_max should be greater than 0");
        result.successful = false;
      }
    }
    if (param.get_name() == "val_min") {
      val_min = param.as_int();
      if (val_min < 1) {
        RCLCPP_WARN(node_->get_logger(), "val_min should be greater than 0");
        result.successful = false;
      }
    }
    if (param.get_name() == "sobel_aperture") {
      const int sobel_aperture = param.as_int();
      if (sobel_aperture < 3 or 7 < sobel_aperture) {
        RCLCPP_WARN(node_->get_logger(), "sobel_aperture should be in range of 3 and 7");
        result.successful = false;
      } else {
        sobel_aperture_ = sobel_aperture;
      }
    }
  }

  if (val_max <= val_min) {
    RCLCPP_WARN(node_->get_logger(), "val_max(%d) should be greater than val_min(%d)", val_max, val_min);
    result.successful = false;
  } else {
    val_max_ = val_max;
    val_min_ = val_min;
  }

  return result;
}


bool IsAvailable(const std::string& type_name) {
  const std::vector<std::string> available_type_names{
    GaussianSpacial::ProcName,
    Diration::ProcName,
    Erosion::ProcName,
    CannyEdge::ProcName,
  };

  bool is_available = false;
  for (auto available_type_name : available_type_names) {
    if (available_type_name == type_name) {
      is_available = true;
    }
  }
  return is_available;
}

Base::SharedPtr Create(std::shared_ptr<rclcpp::Node> node) {
  std::string type_str;
  rclcpp::Parameter type;
  if (!node->get_parameter("type", type)) {
    RCLCPP_WARN(node->get_logger(), "Failed to get type, we use default type gaussian_spacial");
    type_str = GaussianSpacial::ProcName;
  } else {
    type_str = type.as_string();
  }

  Base::SharedPtr ptr;
  if (type_str == GaussianSpacial::ProcName) {
    ptr.reset(new GaussianSpacial(node));
  } else if (type_str == Diration::ProcName) {
    ptr.reset(new Diration(node));
  } else if (type_str == Erosion::ProcName) {
    ptr.reset(new Erosion(node));
  } else if (type_str == CannyEdge::ProcName) {
    ptr.reset(new CannyEdge(node));
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s is not implemented", type_str.c_str());
  }

  return ptr;
}

}
}
