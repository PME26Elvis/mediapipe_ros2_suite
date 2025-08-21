import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2, os, time, mediapipe as mp
import numpy as np

from mediapipe.tasks.python.vision import (
    GestureRecognizer, GestureRecognizerOptions,
    HandLandmarker, HandLandmarkerOptions, RunningMode
)
from mediapipe.tasks.python.core.base_options import BaseOptions

from mediapipe_ros2_node.msg import HandLandmarks, HandGesture, Hand

# 21‑landmark skeleton edges (MediaPipe index convention)
SKELETON_PAIRS = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),
    (0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20),
    (5,9),(9,13),(13,17)
]

class HandNode(Node):
    def __init__(self):
        super().__init__('mediapipe_hand_node')

        # ---------- Parameters ----------
        self.declare_parameter('use_gesture', True)
        self.declare_parameter('use_landmarks', True)
        self.declare_parameter('num_hands', 2)
        self.declare_parameter('min_hand_detection_confidence', 0.5)
        self.declare_parameter('min_hand_presence_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('gesture_model_filename', 'gesture_recognizer.task')
        self.declare_parameter('landmark_model_filename', 'hand_landmarker.task')
        self.declare_parameter('image_topic', '/image_raw')  # v4l2_camera default

        # We publish markers in 'map' to avoid TF complexity for the demo
        self.frame_id = 'map'

        # Resolve model paths from the C++ interfaces package share dir
        share_dir = get_package_share_directory('mediapipe_ros2_node')
        models_dir = os.path.join(share_dir, 'models')
        gesture_model_path  = os.path.join(models_dir, str(self.get_parameter('gesture_model_filename').value))
        landmark_model_path = os.path.join(models_dir, str(self.get_parameter('landmark_model_filename').value))

        # Read params
        self.use_gesture = bool(self.get_parameter('use_gesture').value)
        self.use_landmarks = bool(self.get_parameter('use_landmarks').value)
        self.num_hands = int(self.get_parameter('num_hands').value)
        self.min_det  = float(self.get_parameter('min_hand_detection_confidence').value)
        self.min_pres = float(self.get_parameter('min_hand_presence_confidence').value)
        self.min_track= float(self.get_parameter('min_tracking_confidence').value)
        self.image_topic = str(self.get_parameter('image_topic').value)

        # ---------- Publishers ----------
        self.pub_landmarks = self.create_publisher(HandLandmarks, 'mediapipe/hand_landmarks', 10)
        self.pub_gesture   = self.create_publisher(HandGesture,   'mediapipe/hand_gesture',   10)
        self.pub_markers   = self.create_publisher(MarkerArray,   'mediapipe/markers',        10)
        self.pub_debug_img = self.create_publisher(Image,         'mediapipe/debug_image',    10)

        # ---------- Subscriber ----------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.bridge = CvBridge()

        # keep last BGR frame around for drawing in callbacks
        self.last_bgr = None
        self.last_size = (0, 0)  # (w,h)

        # ---------- MediaPipe initializations ----------
        self.gesture_recognizer = None
        self.hand_landmarker = None

        if self.use_gesture:
            self.gesture_recognizer = GestureRecognizer.create_from_options(
                GestureRecognizerOptions(
                    base_options=BaseOptions(model_asset_path=gesture_model_path),
                    running_mode=RunningMode.LIVE_STREAM,
                    num_hands=self.num_hands,
                    min_hand_detection_confidence=self.min_det,
                    min_hand_presence_confidence=self.min_pres,
                    min_tracking_confidence=self.min_track,
                    result_callback=self._on_gesture_result
                )
            )
            self.get_logger().info(f'Gesture model loaded: {gesture_model_path}')

        if self.use_landmarks:
            self.hand_landmarker = HandLandmarker.create_from_options(
                HandLandmarkerOptions(
                    base_options=BaseOptions(model_asset_path=landmark_model_path),
                    running_mode=RunningMode.LIVE_STREAM,
                    num_hands=self.num_hands,
                    min_hand_detection_confidence=self.min_det,
                    min_hand_presence_confidence=self.min_pres,
                    min_tracking_confidence=self.min_track,
                    result_callback=self._on_landmark_result
                )
            )
            self.get_logger().info(f'Landmarker model loaded: {landmark_model_path}')

        self.get_logger().info('MediaPipe hand node started.')

    # ---------- Image callback ----------
    def on_image(self, msg: Image):
        # keep BGR for drawing; convert a copy to RGB for mediapipe
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        self.last_bgr = bgr
        h, w = bgr.shape[:2]
        self.last_size = (w, h)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        ts_ms = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1e6)
        if ts_ms == 0:
            ts_ms = int(time.time() * 1000)

        if self.gesture_recognizer is not None:
            self.gesture_recognizer.recognize_async(mp_img, ts_ms)
        if self.hand_landmarker is not None:
            self.hand_landmarker.detect_async(mp_img, ts_ms)

    # ---------- MediaPipe callbacks ----------
    def _on_gesture_result(self, result, output_image, timestamp_ms):
        if not result.gestures:
            return
        top = result.gestures[0][0]
        m = HandGesture()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.gesture = top.category_name or top.display_name
        m.score = float(top.score)
        self.pub_gesture.publish(m)
        # Note: drawing is done in landmark callback (we need the 21 points)

    def _on_landmark_result(self, result, output_image, timestamp_ms):
        if not result.hand_landmarks:
            # still publish the last image to keep RViz alive if desired
            self._maybe_publish_debug(self.last_bgr, [])
            return

        # 1) Publish structured ROS messages
        hlm = HandLandmarks()
        hlm.header.stamp = self.get_clock().now().to_msg()
        hlm.header.frame_id = self.frame_id

        ma = MarkerArray()
        marker_id = 0

        for i, hand in enumerate(result.hand_landmarks):
            # package message
            hmsg = Hand()
            if result.handedness and i < len(result.handedness) and result.handedness[i]:
                hd = result.handedness[i][0]
                hmsg.handedness = hd.category_name or (hd.display_name or '')
                hmsg.score = float(hd.score)
            else:
                hmsg.handedness = ''
                hmsg.score = 0.0

            for lm in hand:
                hmsg.landmarks.append(Point(x=float(lm.x), y=float(lm.y), z=float(lm.z)))
            hlm.hands.append(hmsg)

            # points marker
            pts = Marker()
            pts.header.frame_id = self.frame_id
            pts.header.stamp = self.get_clock().now().to_msg()
            pts.ns = 'hand_landmarks'; pts.id = marker_id; marker_id += 1
            pts.type = Marker.SPHERE_LIST; pts.action = Marker.ADD
            pts.scale.x = pts.scale.y = pts.scale.z = 0.02
            pts.color.a = 1.0
            handed = hmsg.handedness.lower()
            if handed.startswith('left'):  pts.color.g = 1.0
            elif handed.startswith('right'): pts.color.r = 1.0
            else: pts.color.b = 1.0
            for lm in hand:
                pts.points.append(Point(x=float(lm.x), y=float(lm.y), z=float(lm.z)))
            ma.markers.append(pts)

            # skeleton lines
            lines = Marker()
            lines.header.frame_id = self.frame_id
            lines.header.stamp = self.get_clock().now().to_msg()
            lines.ns = 'hand_skeleton'; lines.id = marker_id; marker_id += 1
            lines.type = Marker.LINE_LIST; lines.action = Marker.ADD
            lines.scale.x = 0.01
            lines.color.a = 1.0; lines.color.r = lines.color.g = lines.color.b = 1.0
            for a, b in SKELETON_PAIRS:
                lines.points.append(Point(x=float(hand[a].x), y=float(hand[a].y), z=float(hand[a].z)))
                lines.points.append(Point(x=float(hand[b].x), y=float(hand[b].y), z=float(hand[b].z)))
            ma.markers.append(lines)

        self.pub_landmarks.publish(hlm)
        self.pub_markers.publish(ma)

        # 2) Publish annotated debug image (YOLO‑style overlay)
        self._maybe_publish_debug(self.last_bgr, result.hand_landmarks, result.handedness)

    # ---------- Drawing / debug image ----------
    def _maybe_publish_debug(self, bgr, hands, handedness_list=None):
        if bgr is None:
            return
        dbg = bgr.copy()
        w, h = self.last_size

        # Draw each hand: landmarks, skeleton, and a simple bounding box
        for i, hand in enumerate(hands):
            color = (0,255,0)  # default green
            if handedness_list and i < len(handedness_list) and handedness_list[i]:
                name = (handedness_list[i][0].category_name or '').lower()
                if name.startswith('right'): color = (0,0,255)  # red
                elif name.startswith('left'): color = (0,255,0)  # green
            pts_px = []
            for lm in hand:
                x = int(float(lm.x) * w)
                y = int(float(lm.y) * h)
                pts_px.append((x,y))
                cv2.circle(dbg, (x,y), 3, color, -1, lineType=cv2.LINE_AA)
            # skeleton
            for a,b in SKELETON_PAIRS:
                cv2.line(dbg, pts_px[a], pts_px[b], (255,255,255), 2, lineType=cv2.LINE_AA)
            # bbox
            xs = [p[0] for p in pts_px]; ys = [p[1] for p in pts_px]
            cv2.rectangle(dbg, (min(xs),min(ys)), (max(xs),max(ys)), color, 2)

        # Publish
        img_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'map'
        self.pub_debug_img.publish(img_msg)

def main():
    rclpy.init()
    rclpy.spin(HandNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
