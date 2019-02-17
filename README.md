# image_proc_chain

Way to validate result of image process every step.

- Provide library that is reconfiguable image process.

  Reconfigulation function is powered by ROS dynamic reconfigure class

  <http://wiki.ros.org/dynamic_reconfigure>

- Provide chain filter node for validation of process function.

  Using ROS nodelet module

  <http://wiki.ros.org/nodelet>


# Processors

| name | link |
| ---- | ---- |
| canny_edge | <https://en.wikipedia.org/wiki/Canny_edge_detector> |
| gaussian | <https://en.wikipedia.org/wiki/Gaussian_filter> |
| bilateral | <https://en.wikipedia.org/wiki/Bilateral_filter> |

# How to use

  ```bash
  roslaunch image_proc_chain chain_filter.launch --screen input_topic:=/cv_camera/image_raw default_chain_num:=3
  ```
