# image_proc_chain

Way to validate result of image process every step.

- Provide library that is reconfiguable image process.

  Reconfigulation function is powered by ROS dynamic reconfigure class

  <http://wiki.ros.org/dynamic_reconfigure>

- Provide chained process node for validation of process function.

  Using ROS nodelet module

  <http://wiki.ros.org/nodelet>

# Operation environment

- Ubuntu 16.04 LTS
- ROS kinetic 1.12.7

# Overview

![overview](https://github.com/fugashy/image_proc_chain/blob/images/overview.png)

# Processors

  - Bilateral space filter
  - Canny edge detection
  - Dilation
  - Erosion
  - Gamma correction of gray image
  - Gaussian space filter


# How to use

  - Single process

  ```bash
  roslaunch image_proc_chain single_process.launch --screen input_topic:=/sensor_msgs/image/topic
  ```

  - Chain process

  ```bash
  roslaunch image_proc_chain chain_process.launch --screen input_topic:=/sensor_msgs/image/topic default_chain_num:=3
  ```

  Then, you can select type of process by using ROS service.

  ```bash
  rosservice call /chain_process_node/filter_000 "type: 'canny_edge'"
  ```

  Also, you can select parameter of the process by using dynamic-reconfigure.

  ```bash
  rosrun rqt_reconfigure rqt_reconfigure
  ```

  Number of chain is variable by using ROS service.

  ```bash
  rosservice call /chain_process_node/change_chain_num "num: 4"
  ```

# Known instability

- If you set a large value for the chain length, the flow of the topic may be interrupted halfway.

# Lisence

Under 3-caused BSD lisence.
