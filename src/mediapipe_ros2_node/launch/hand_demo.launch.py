from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    py_node = Node(
        package='mediapipe_ros2_py',
        executable='hand_node',
        name='mediapipe_hand_node',
        parameters=[{
            'use_gesture': True,
            'use_landmarks': True,
            'num_hands': 2,
            'image_topic': '/camera/image_raw',  # 依你的相機 topic 調整
        }]
    )

    # 你也可以在這裡加上 camera 節點或 RViz
    return LaunchDescription([
        py_node
    ])
