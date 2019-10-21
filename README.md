# image_proc_chain

Way to validate result of image process every step.

# Develop environment

- OS: MacOS Catalina 10.15
- ROS2 dashing diademata (20190531)

# Target

![overview](https://github.com/fugashy/image_proc_chain/blob/images/overview.png)

# Processors

  - ~~Bilateral space filter~~
  - Canny edge detection
  - Dilation
  - Erosion
  - ~~Gamma correction of gray image~~
  - Gaussian spacial filter


# How to use

T.B.Developed

- As single chained image processor

```bash
ros2 run image_proc_chain image_proc_chain image_proc_chain_chain_piece_node /chain_piece/image_in:=/your_image_topic_name
```

  Switching command is

  ```bash
  ros2 service call /chain_piece/switch_processor_type image_proc_chain/srv/SwitchProcessorType "'type': 'canny_edge'"
  ```

# Lisence

Under 3-caused BSD lisence.
