# ~/ros2_ws/src/mediapipe_ros2_py/launch/mp_node.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_arg         = DeclareLaunchArgument('model', default_value='hand')   # hand|pose|face
    image_topic_arg   = DeclareLaunchArgument('image_topic', default_value='/image_raw')
    topic_prefix_arg  = DeclareLaunchArgument('topic_prefix', default_value='/mediapipe')
    frame_id_arg      = DeclareLaunchArgument('frame_id', default_value='map')
    use_gesture_arg   = DeclareLaunchArgument('use_gesture', default_value='true')
    debug_img_arg     = DeclareLaunchArgument('publish_debug_image', default_value='true')

    start_rviz_arg    = DeclareLaunchArgument('start_rviz', default_value='false')
    rviz_config_arg   = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('mediapipe_ros2_node'),
            'rviz', 'mediapipe_default.rviz'))

    model        = LaunchConfiguration('model')
    image_topic  = LaunchConfiguration('image_topic')
    topic_prefix = LaunchConfiguration('topic_prefix')
    frame_id     = LaunchConfiguration('frame_id')
    use_gesture  = LaunchConfiguration('use_gesture')
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    start_rviz   = LaunchConfiguration('start_rviz')
    rviz_config  = LaunchConfiguration('rviz_config')

    mp_node = Node(
        package='mediapipe_ros2_py',
        executable='mp_node',
        name='mp_node',
        output='screen',
        parameters=[{
            'model': model,
            'image_topic': image_topic,
            'topic_prefix': topic_prefix,
            'frame_id': frame_id,
            'use_gesture': use_gesture,
            'publish_debug_image': publish_debug_image,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        model_arg, image_topic_arg, topic_prefix_arg, frame_id_arg,
        use_gesture_arg, debug_img_arg, start_rviz_arg, rviz_config_arg,
        mp_node, rviz_node
    ])
