import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_PKG_NAME = "image_proc_chain"


def generate_launch_description():
    io_components = [
        ComposableNode(
            package="topic_tools",
            plugin="topic_tools::RelayNode",
            namespace=f"/{_PKG_NAME}/io",
            name="input",
            parameters=[{
                "input_topic": f"/{_PKG_NAME}/io/input_image",
                "output_topic": f"/{_PKG_NAME}/pieces/no_0/input_image"}]),
        ComposableNode(
            package="topic_tools",
            plugin="topic_tools::RelayNode",
            namespace=f"/{_PKG_NAME}/io",
            name="output",
            parameters=[{
                "input_topic": f"/{_PKG_NAME}/pieces/no_0/input_image",
                "output_topic": f"/{_PKG_NAME}/io/output_image"}]),
            ]
    ipc_container = ComposableNodeContainer(
        package=_PKG_NAME,
        namespace=_PKG_NAME,
        name="component_container",
        executable=f"{_PKG_NAME}_component_container",
        composable_node_descriptions=io_components,
        output='log')

    return launch.LaunchDescription([ipc_container])
