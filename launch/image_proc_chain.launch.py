import launch
from launch_ros.actions import ComposableNodeContainer

_PKG_NAME = "image_proc_chain"


def generate_launch_description():
    container = ComposableNodeContainer(
        name="component_container",
        namespace=_PKG_NAME,
        package=_PKG_NAME,
        executable=f"{_PKG_NAME}_component_container",
        output='log')

    return launch.LaunchDescription([container])
