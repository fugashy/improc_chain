// Copyright 2019 fugashy
#include <chrono>
#include <memory>
#include <ratio>
#include <string>
#include <vector>

#include <image_proc_chain/image_processors.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>


using std::placeholders::_1;

namespace image_proc_chain {
namespace image_processors {


Base::Base(rclcpp::Node* node) : node_(node) {}

const char GaussianSpacial::ProcName[] = "gaussian_spacial";

GaussianSpacial::GaussianSpacial(rclcpp::Node* node)
    : Base(node),
      kernel_(cv::Size(21, 21)),
      sigma_(cv::Vec2d(1.0, 1.0)),
      iteration_count_(1)  {
  param_handler_ = node->add_on_set_parameters_callback(std::bind(&GaussianSpacial::ChangeParameters, this, _1));
  node->declare_parameter(
      "kernel_x",
      21,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("kernel_x")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The size of kernel in x axis")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(101)
                .step(2)
            })));
  node->declare_parameter(
      "kernel_y",
      21,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("kernel_y")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The size of kernel in y axis")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(101)
                .step(2)
            })));
  node->declare_parameter(
      "sigma_x",
      1.0,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("signal_x")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
        .description("The signal of gaussian in x axis")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::FloatingPointRange>()
                .from_value(1.0)
                .to_value(100.0)
                .step(0.001)
            }))
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>()));
  node->declare_parameter(
      "sigma_y",
      1.0,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("signal_y")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
        .description("The signal of gaussian in y axis")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::FloatingPointRange>()
                .from_value(1.0)
                .to_value(100.0)
                .step(0.001)
            }))
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>()));
  node->declare_parameter(
      "iteration_count",
      1,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("iteration_count")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The number of iteration of this filter")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(100)
                .step(1)
            })));
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
      if (width % 2 != 1 || width < 1) {
        result.successful = false;
      } else {
        kernel_.width = param.as_int();
      }
    }
    if (param.get_name() == "kernel_y") {
      const int height = param.as_int();
      if (height % 2 != 1 || height < 1) {
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

const char Diration::ProcName[] = "diration";

Diration::Diration(rclcpp::Node* node)
    : Base(node),
      kernel_(cv::Mat::ones(3, 3, CV_8UC1)),
      iteration_count_(1) {
  param_handler_ = node->add_on_set_parameters_callback(std::bind(&Diration::ChangeParameters, this, _1));
  node->declare_parameter(
      "kernel_size",
      3,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("kernal_zie")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The size of kernel")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(100)
                .step(1)
            })));
  node->declare_parameter(
      "iteration_count",
      1,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("iteration_count")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The number of iteration of this filter")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(100)
                .step(1)
            })));
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

const char Erosion::ProcName[] = "erosion";

Erosion::Erosion(rclcpp::Node* node)
    : Base(node),
      kernel_(cv::Mat::ones(3, 3, CV_8UC1)),
      iteration_count_(1) {
  param_handler_ = node->add_on_set_parameters_callback(std::bind(&Erosion::ChangeParameters, this, _1));
  node->declare_parameter(
      "kernel_size",
      3,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("kernal_zie")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The size of kernel")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(100)
                .step(1)
            })));
  node->declare_parameter(
      "iteration_count",
      1,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("iteration_count")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The number of iteration of this filter")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(100)
                .step(1)
            })));
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

const char CannyEdge::ProcName[] = "canny_edge";

CannyEdge::CannyEdge(rclcpp::Node* node)
    : Base(node),
      val_max_(100),
      val_min_(50),
      sobel_aperture_(3) {
  param_handler_ = node->add_on_set_parameters_callback(std::bind(&CannyEdge::ChangeParameters, this, _1));
  node->declare_parameter(
      "val_max",
      100,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("val_max")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The max value of filter")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(2)
                .to_value(255)
                .step(1)
            })));
  node->declare_parameter(
      "val_min",
      50,
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("val_min")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The min value of filter")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(1)
                .to_value(254)
                .step(1)
            })));
  RCLCPP_ERROR(node->get_logger(), "try to declare sobel_aperture");
  node->declare_parameter(
      "sobel_aperture",
      static_cast<int>(sobel_aperture_),
      rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
        .name("sobel_aperture")
        .type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        .description("The size of sobel aperture")
        .additional_constraints("")
        .read_only(false)
        .dynamic_typing(false)
        .floating_point_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1>())
        .integer_range(
          rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1>(
            {
              rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
                .from_value(3)
                .to_value(7)
                .step(2)
            })));
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
  result.successful = false;
  RCLCPP_DEBUG(node_->get_logger(), "Parameter event has called");

  for (auto param : params) {
    if (param.get_name() == "val_max") {
      const uint32_t v = static_cast<uint32_t>(param.as_int());
      if (v <= val_min_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "val_max should be greater than val_min(%d)",
            val_min_);
        continue;
      }
      val_max_ = v;
      result.successful = true;
    }
    if (param.get_name() == "val_min") {
      const uint32_t v = static_cast<uint32_t>(param.as_int());
      if (v >= val_max_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "val_min should be smaller than val_max(%d)",
            val_max_);
        continue;
      }
      val_min_ = v;
      result.successful = true;
    }
    if (param.get_name() == "sobel_aperture") {
      sobel_aperture_ = param.as_int();
      result.successful = true;
    }
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
    if (available_type_name == type_name.c_str()) {
      is_available = true;
    }
  }
  return is_available;
}

Base::SharedPtr Create(rclcpp::Node* node) {
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

}  // namespace image_processors
}  // namespace image_proc_chain
