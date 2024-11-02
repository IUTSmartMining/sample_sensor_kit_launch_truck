from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context):
    camera_name = LaunchConfiguration('camera_name')
    camera_info_url = Path(get_package_share_directory('common_sensor_launch')) / 'config' / f'{camera_name.perform(context)}_info.yaml'
    frame_id = f'{camera_name.perform(context)}/camera_link'
    rtsp_url = LaunchConfiguration('rtsp_url')
    gscam_config = f'rtspsrc location={rtsp_url.perform(context)} latency=0 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! videoscale'
    
    container = ComposableNodeContainer(
        name='gscam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gscam2',
                plugin='gscam2::GSCamNode',
                name='gscam_publisher',
                parameters=[
                    {'camera_name': camera_name},
                    {'camera_info_url': f'file://{camera_info_url}'},
                    {'frame_id': frame_id},
                    {'gscam_config': gscam_config},
                ],
                remappings=[
                    ('image_raw', 'image_rect_color'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ],
        output='screen',
        respawn=True,
        respawn_delay=0.1,
    )

    return [container]

def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument('camera_name')
    rtsp_url_arg = DeclareLaunchArgument('rtsp_url')
    
    return LaunchDescription([
        camera_name_arg, rtsp_url_arg,
        OpaqueFunction(function=launch_setup),
    ])
