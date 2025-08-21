from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        parameters=[{
            # 'video_device': '/dev/video0',
            # 'frame_rate': 30,
        }]
    )

    mp = Node(
        package='mediapipe_ros2_py',
        executable='mp_node',
        name='mediapipe_node',
        output='screen',
        parameters=[{
            'model': 'hand',           # change to 'pose' or 'face'
            'image_topic': '/image_raw',
            'topic_prefix': '/mediapipe',
            'publish_debug_image': True,
        }]
    )

    return LaunchDescription([cam, mp])