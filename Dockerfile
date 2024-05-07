from ros:humble

env ROS_DISTRO humble

run apt-get update && \
  apt-get install -y \
    libopencv-dev \
    python3-opencv \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-topic-tools

workdir /opt/ipc/src
copy . /opt/ipc/src/image_proc_chain/
run git clone --depth=1 https://github.com/fugashy/image_proc_chain_msgs.git
workdir /opt/ipc
run bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

workdir /
copy ./entrypoint.sh /
entrypoint ["/entrypoint.sh"]
