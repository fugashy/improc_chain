import launch
import launch_ros

_PKG_NAME = 'image_proc_chain'


def generate_launch_description():
    chain_num = launch.substitutions.LaunchConfiguration('chain_num')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'chain_num',
            default_value='3',
            description='Number of image processor(chain_piece)'),
        launch_ros.actions.Node(
            package=_PKG_NAME,
            node_executable=_PKG_NAME+'_chain_processor_node',
            node_name='chain_processor',
            output='screen',
            parameters=[{'chain_num': chain_num}])])
