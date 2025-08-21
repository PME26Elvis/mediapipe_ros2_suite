from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    num_hands   = LaunchConfiguration('num_hands')
    use_rviz    = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('num_hands',   default_value='2'),
        DeclareLaunchArgument('use_rviz',    default_value='true'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam',
            # 需要時可加參數：parameters=[{'frame_rate': 30}]
        ),

        Node(
            package='mediapipe_ros2_py',
            executable='hand_node',
            name='mediapipe_hand_node',
            parameters=[{
                'use_gesture': True,
                'use_landmarks': True,
                'num_hands': num_hands,
                'image_topic': image_topic,
            }]
        ),

        # 可選的 RViz（用 group_action/conditional 也行；這裡簡單起見直接常開或 CLI 控制）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',
                ['$(ros2 pkg prefix mediapipe_ros2_node)/share/mediapipe_ros2_node/rviz/hand_demo.rviz']
            ],
            condition=None  # 想用條件可改成 IfCondition(use_rviz)
        ),
    ])
