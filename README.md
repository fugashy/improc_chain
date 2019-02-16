# image_proc_chain

- Provide reconfiguable filter library.

  Reconfigulation function is powered by ROS dynamic reconfigure class

  <http://wiki.ros.org/dynamic_reconfigure>

- Provide chain filter node for validation of filter function.


# Filters

| name | link |
| ---- | ---- |
| canny_edge | <https://en.wikipedia.org/wiki/Canny_edge_detector> |
| gaussian | <https://en.wikipedia.org/wiki/Gaussian_filter> |
| bilateral | <https://en.wikipedia.org/wiki/Bilateral_filter> |
| pass_through | Do nothing |

# How to use chain filter

  ```bash
  roslaunch image_proc_chain chain_filter.launch --screen input_topic:=/cv_camera/image_raw default_chain_num:=3
  ```
