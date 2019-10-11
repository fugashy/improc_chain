#ifndef IMAGE_PROC_CHAIN_IO_HPP_
#define IMAGE_PROC_CHAIN_IO_HPP_
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_proc_chain {

class IO : public rclcpp::Node {
 public:
  using SharedPtr = std::shared_ptr<IO>;

  IO(const std::string& node_name);

 private:
  void Process(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}

#endif
