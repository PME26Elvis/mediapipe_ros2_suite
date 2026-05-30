from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    py_node = Node(
        package='mediapipe_ros2_py',
        executable='mp_node',
        name='mediapipe_hand_node',
        output='screen',
        parameters=[{
            'model': 'hand',
            'image_topic': '/camera/image_raw',  # 依你的相機 topic 調整
            'topic_prefix': '/mediapipe',
            'use_gesture': True,
            'publish_debug_image': True,
        }]
    )

    # 你也可以在這裡加上 camera 節點或 RViz
    return LaunchDescription([
        py_node
    ])
