from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    camera_name = LaunchConfiguration('camera_name')
    camera_info_url = f'package://common_sensor_launch/config/{camera_name.perform(context)}_info.yaml'
    frame_id = f'{camera_name.perform(context)}/camera_link'
    rtsp_url = LaunchConfiguration('rtsp_url')
    gscam_config = f'rtspsrc location={rtsp_url.perform(context)} latency=0 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! videoscale'

    node = ComposableNodeContainer(
        name='gscam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            # GSCam driver
            ComposableNode(
                package='gscam',
                plugin='gscam::GSCam',
                name='gscam_node',
                parameters=[{
                    'gscam_config': gscam_config,
                    'camera_info_url': camera_info_url,
                    'camera_name': 'camera0',
                    'frame_id': 'camera',
                    'reopen_on_eof': True,
                    'use_sensor_data_qos': True,
                }],
                # Future-proof: enable zero-copy IPC when it is available
                # https://github.com/ros-perception/image_common/issues/212
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # Bayer color decoding
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer_node',
                # namespace='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # Mono rectification
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='mono_rectify_node',
                # namespace='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('image', 'image_mono'),
                    ('image_rect', 'image_rect_mono'),
                ],
            ),

            # Color rectification
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='color_rectify_node',
                # namespace='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('image', 'image_color'),
                    ('image_rect', 'image_rect_color'),
                ],
            ),
        ],
        output='screen',
    )

    return [node]

def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument('camera_name')
    rtsp_url_arg = DeclareLaunchArgument('rtsp_url')
    
    return LaunchDescription([
        camera_name_arg, rtsp_url_arg,
        OpaqueFunction(function=launch_setup),
    ])
