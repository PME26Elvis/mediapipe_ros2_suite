from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cam = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        parameters=[{
            # 'video_device': '/dev/video0',  # 需要時打開
            # 'frame_rate': 30,               # 需要時打開
        }]
    )

    mp = Node(
        package='mediapipe_ros2_py',
        executable='mp_node',
        name='mediapipe_hand_node',
        output='screen',
        parameters=[{
            'model': 'hand',
            'image_topic': '/image_raw',   # ★ 改這裡，對齊 v4l2_camera 預設
            'topic_prefix': '/mediapipe',
            'use_gesture': True,
            'publish_debug_image': True,
        }]
    )

    return LaunchDescription([cam, mp])
