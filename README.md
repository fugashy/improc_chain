# image_proc_chain

Way to validate result of image process every step.

- Provide library that is reconfiguable image process.

  Reconfigulation function is powered by ROS dynamic reconfigure class

  <http://wiki.ros.org/dynamic_reconfigure>

- Provide chain filter node for validation of process function.

  Using ROS nodelet module

  <http://wiki.ros.org/nodelet>


# Processors

  - Gaussian space filter
  - Bilateral space filter
  - Gamma correction of gray image
  - Canny edge detection


# How to use

  - Single process

  ```bash
  roslaunch image_proc_chain single_process.launch --screen input_topic:=/sensor_msgs/image/topic
  ```

  - Chain process

  ```bash
  roslaunch image_proc_chain chain_process.launch --screen input_topic:=/sensor_msgs/image/topic default_chain_num:=3
  ```
