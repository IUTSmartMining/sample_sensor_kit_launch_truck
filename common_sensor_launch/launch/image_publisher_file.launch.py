from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    rtsp_url = LaunchConfiguration('rtsp_url')
    camera_info_url = LaunchConfiguration('camera_info_url')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    frame_id = LaunchConfiguration('frame_id')
    retry = LaunchConfiguration('retry', default='true')

    return LaunchDescription([
        
        DeclareLaunchArgument('rtsp_url', description='RTSP URL to the camera'),
        DeclareLaunchArgument('camera_info_url', description='URL to the camera info file'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time if true'),
        DeclareLaunchArgument('frame_id', description='Camera frame ID'),
        DeclareLaunchArgument('retry', default_value='true',
                              description='Retry after timeout'),

        launch_ros.actions.Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher',
            output='screen',
            respawn=True,
            respawn_delay=0.1,
            arguments=[rtsp_url],
            parameters=[
                {'camera_info_url': camera_info_url},
                {'use_sim_time': use_sim_time},
                {'frame_id': frame_id},
                {'retry': retry},
            ]),
    ])
