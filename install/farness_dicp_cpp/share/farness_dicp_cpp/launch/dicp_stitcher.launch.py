from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        'share', 'farness_dicp_cpp'
    )

    return LaunchDescription([
        Node(
            package='farness_dicp_cpp',
            executable='main_node',
            name='doppler_icp_stitcher',
            output='screen',
            
             parameters=[
                '/home/farness/ws_dicp_cpp/src/farness_dicp_cpp/config/dicp_params.yaml' ],

        )
    ])
