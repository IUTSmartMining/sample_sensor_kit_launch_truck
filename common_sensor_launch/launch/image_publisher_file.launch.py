from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    rtsp_url = LaunchConfiguration('rtsp_url')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        
        DeclareLaunchArgument('rtsp_url', description='RTSP URL to the camera'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time if true'),
        DeclareLaunchArgument('frame_id', description='Camera frame ID'),

        launch_ros.actions.Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher',
            output='screen',
            respawn=True,
            respawn_delay=0.1,
            arguments=[rtsp_url],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'frame_id': frame_id}
            ]),
    ])
