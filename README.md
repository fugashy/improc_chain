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

- As single chained image processor

```bash
ros2 run image_proc_chain image_proc_chain image_proc_chain_chain_piece_node /chain_piece/image_in:=/your_image_topic_name
```

Switching command is

```bash
ros2 service call /chain_piece/switch_processor_type image_proc_chain/srv/SwitchProcessorType "'type': 'canny_edge'"
```

- As chained image processor

```bash
ros2 run image_proc_chain_chain_processor_node /camera/image_raw:=/your_image_topic_name
```

Switching command is

```bash
ros2 service call /chain_piece_N/switch_processor_type image_proc_chain/srv/SwitchProcessorType "'type': 'canny_edge'"
```

N represents the number of chain piece.

Command of changing num is

```bash
ros2 service call /chain_processor/change_chain_num image_proc_chain/srv/ChangeChainNum "num: 4"
```


# Lisence

Under 3-claused BSD lisence.
