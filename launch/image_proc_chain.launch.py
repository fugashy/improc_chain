import launch
from launch_ros.actions import ComposableNodeContainer

_PKG_NAME = "image_proc_chain"


def generate_launch_description():
    ipc_container = ComposableNodeContainer(
        package=_PKG_NAME,
        namespace=_PKG_NAME,
        name="component_container",
        executable=f"{_PKG_NAME}_component_container",
        output='log')

    return launch.LaunchDescription([ipc_container])
