from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam1 = Node(package='v4l2_camera', executable='v4l2_camera_node', name='cam1',
                parameters=[{'video_device': '/dev/video0'}])
    cam2 = Node(package='v4l2_camera', executable='v4l2_camera_node', name='cam2',
                parameters=[{'video_device': '/dev/video2'}])

    mp1 = Node(package='mediapipe_ros2_py', executable='mp_node', name='mp_cam1',
               parameters=[{'model':'hand','image_topic':'/cam1/image_raw','topic_prefix':'/mediapipe/cam1'}])
    mp2 = Node(package='mediapipe_ros2_py', executable='mp_node', name='mp_cam2',
               parameters=[{'model':'pose','image_topic':'/cam2/image_raw','topic_prefix':'/mediapipe/cam2'}])

    return LaunchDescription([cam1, cam2, mp1, mp2])
