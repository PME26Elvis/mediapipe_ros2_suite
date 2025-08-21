from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    model = LaunchConfiguration('model', default='hand')
    image_topic = LaunchConfiguration('image_topic', default='/image_raw')
    topic_prefix = LaunchConfiguration('topic_prefix', default='/mediapipe')

    return LaunchDescription([
        DeclareLaunchArgument('model'),
        DeclareLaunchArgument('image_topic'),
        DeclareLaunchArgument('topic_prefix'),
        Node(
            package='mediapipe_ros2_py',
            executable='mp_node',
            name='mediapipe_node',
            output='screen',
            parameters=[{
                'model': model,
                'image_topic': image_topic,
                'topic_prefix': topic_prefix,
                'publish_debug_image': True,
            }]
        )
    ])