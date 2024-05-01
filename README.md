# image_proc_chain

Way to validate result of image process every step.

Simple image processors(chain_piece) are chained by Pub/Sub.
I named this as image_proc_chain.

chain_piece has:
- parameters that are reconfigurable by using ros parameters.
(ex: kernel size of gaussian filter)
- a ros service that can switch the type of image processor.
(ex: gaussian spacial filter, canny edge detection, etc...)

image_proc_chain has a ros service that can change length of the chain_piece. 

# Develop environment

- ROS2 humble

# Overview

![overview](https://github.com/fugashy/image_proc_chain/blob/images/overview.png)

# Processors

  - ~~Bilateral space filter~~
  - Canny edge detection
  - Dilation
  - Erosion
  - ~~Gamma correction of gray image~~
  - Gaussian spacial filter


# How to use

## Build this packages with dependencies

```bash
mkdir ~/ipc_ws/src -p
cd ~/ipc_ws/src
git clone -b feature/humble https://github.com/fugashy/image_proc_chain.git
git clone https://github.com/fugashy/image_proc_chain_msgs.git
cd ~/ipc_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Run

```bash
cd ~/ipc_ws
source install/setup.bash
ros2 launch image_proc_chain image_proc_chain.launch.py
```

## Change length of the chain

```bash
cd ~/ipc_ws
source install/setup.bash
ros2 service call /image_proc_chain/component_container/change_length image_proc_chain_msgs/srv/ChangeChainNum "{num: 3}"
```

## Change type of image processors

Switch the type from gaussian to erosion

```bash
cd ~/ipc_ws
source install/setup.bash
ros2 service call /image_proc_chain/pieces/no_1/switch_processor_type image_proc_chain_msgs/srv/SwitchProcessorType "type: 'erosion'"
```

# Dockerfile

```bash
docker build -t ipc:humble .
docker run -it --rm ipc:humble ros2 launch image_proc_chain image_proc_chain.launch.py
```

# Lisence

Under 3-claused BSD lisence.
