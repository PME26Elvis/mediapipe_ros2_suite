import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mediapipe_ros2_interfaces.msg import HandGesture

MAP = {
    'Open_Palm':  ('stop', 0.0, 0.0),
    'Closed_Fist':('go',   0.8, 0.0),
    'Thumb_Up':   ('turn', 0.0, 1.0),
    'Thumb_Down': ('turn', 0.0,-1.0),
    'Victory':    ('go',   0.6, 0.6),
}

class Gesture2Turtle(Node):
    def __init__(self):
        super().__init__('gesture_to_turtlesim')
        self.declare_parameter('gesture_topic', '/mediapipe/hand/gesture')
        self.declare_parameter('cmd_topic', '/turtle1/cmd_vel')
        self.declare_parameter('score_threshold', 0.6)

        self.sub = self.create_subscription(
            HandGesture,
            self.get_parameter('gesture_topic').value,
            self.on_gesture, 10
        )
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.score_th = float(self.get_parameter('score_threshold').value)
        self.get_logger().info('Gesture→Turtlesim bridge started.')

    def on_gesture(self, msg: HandGesture):
        if msg.score < self.score_th:
            return
        name = msg.gesture or ''
        if name not in MAP:
            return
        mode, lin, ang = MAP[name]
        t = Twist(); t.linear.x = lin; t.angular.z = ang
        self.pub.publish(t)
        self.get_logger().info(f'[{name}] -> lin={lin:.2f}, ang={ang:.2f}')


def main():
    rclpy.init(); rclpy.spin(Gesture2Turtle()); rclpy.shutdown()

if __name__ == '__main__':
    main()