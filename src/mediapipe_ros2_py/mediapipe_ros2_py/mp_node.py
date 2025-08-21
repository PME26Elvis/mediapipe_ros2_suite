import os, time, cv2, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge

import mediapipe as mp
from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python.vision import (
    GestureRecognizer, GestureRecognizerOptions,
    HandLandmarker, HandLandmarkerOptions,
    PoseLandmarker, PoseLandmarkerOptions,
    FaceLandmarker, FaceLandmarkerOptions,
    RunningMode
)
from mediapipe_ros2_interfaces.msg import (
    HandLandmarks, HandGesture, Hand,
    FaceLandmarks, PoseLandmarks
)
# --- Hand skeleton (21 pts) ---
HAND_EDGES = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),
    (0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20),
    (5,9),(9,13),(13,17)
]
# --- Minimal BlazePose edges (33 pts); feel free to expand ---
POSE_EDGES = [
    (11,12), (11,13), (13,15), (12,14), (14,16),
    (23,24), (11,23), (12,24), (23,25), (25,27), (24,26), (26,28)
]
class MPNode(Node):
    def __init__(self):
        super().__init__('mediapipe_node')
        # -------- Parameters --------
        self.declare_parameter('model', 'hand')           # 'hand' | 'pose' | 'face'
        self.declare_parameter('use_gesture', True)       # only for hand
        self.declare_parameter('num_hands', 2)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('topic_prefix', '/mediapipe')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('output_coord_mode', 'normalized') # 'normalized' | 'pixel'
        # confidence thresholds
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_presence_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        # model filenames (expected under share/<pkg>/models)
        self.declare_parameter('gesture_model_filename', 'gesture_recognizer.task')
        self.declare_parameter('hand_model_filename', 'hand_landmarker.task')
        self.declare_parameter('pose_model_filename', 'pose_landmarker.task')
        self.declare_parameter('face_model_filename', 'face_landmarker.task')

        # -------- Read params --------
        self.model = str(self.get_parameter('model').value).lower()
        self.use_gesture = bool(self.get_parameter('use_gesture').value)
        self.num_hands = int(self.get_parameter('num_hands').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.topic_prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.pub_dbg = bool(self.get_parameter('publish_debug_image').value)
        self.coord_mode = str(self.get_parameter('output_coord_mode').value)
        self.min_det = float(self.get_parameter('min_detection_confidence').value)
        self.min_pres = float(self.get_parameter('min_presence_confidence').value)
        self.min_trk = float(self.get_parameter('min_tracking_confidence').value)

        # resolve models dir via ament index (share dir of message pkg)
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('mediapipe_ros2_node')
        models_dir = os.path.join(share_dir, 'models')
        self.paths = {
            'gesture': os.path.join(models_dir, str(self.get_parameter('gesture_model_filename').value)),
            'hand':    os.path.join(models_dir, str(self.get_parameter('hand_model_filename').value)),
            'pose':    os.path.join(models_dir, str(self.get_parameter('pose_model_filename').value)),
            'face':    os.path.join(models_dir, str(self.get_parameter('face_model_filename').value)),
        }

        # -------- Publishers --------
        self.pub_markers = self.create_publisher(MarkerArray, f'{self.topic_prefix}/markers', 10)
        self.pub_debug   = self.create_publisher(Image,       f'{self.topic_prefix}/debug_image', 10) if self.pub_dbg else None

        # per-model publishers
        self.pub_hand_lm = self.create_publisher(HandLandmarks, f'{self.topic_prefix}/hand/landmarks', 10)
        self.pub_gesture = self.create_publisher(HandGesture,   f'{self.topic_prefix}/hand/gesture',   10)
        self.pub_pose_lm = self.create_publisher(PoseLandmarks, f'{self.topic_prefix}/pose/landmarks', 10)
        self.pub_face_lm = self.create_publisher(FaceLandmarks, f'{self.topic_prefix}/face/landmarks', 10)

        # -------- Subscriber --------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.bridge = CvBridge()
        self.last_bgr = None
        self.last_size = (0,0)  # (w, h)

        # -------- MediaPipe initializations --------
        self.gesture = None
        self.hand = None
        self.pose = None
        self.face = None

        if self.model == 'hand':
            if self.use_gesture:
                self.gesture = GestureRecognizer.create_from_options(GestureRecognizerOptions(
                    base_options=BaseOptions(model_asset_path=self.paths['gesture']),
                    running_mode=RunningMode.LIVE_STREAM,
                    num_hands=self.num_hands,
                    min_hand_detection_confidence=self.min_det,
                    min_hand_presence_confidence=self.min_pres,
                    min_tracking_confidence=self.min_trk,
                    result_callback=self._on_gesture
                ))
                self.get_logger().info(f'Gesture model: {self.paths["gesture"]}')
            self.hand = HandLandmarker.create_from_options(HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=self.paths['hand']),
                running_mode=RunningMode.LIVE_STREAM,
                num_hands=self.num_hands,
                min_hand_detection_confidence=self.min_det,
                min_hand_presence_confidence=self.min_pres,
                min_tracking_confidence=self.min_trk,
                result_callback=self._on_hand
            ))
            self.get_logger().info(f'Hand model: {self.paths["hand"]}')

        elif self.model == 'pose':
            self.pose = PoseLandmarker.create_from_options(PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=self.paths['pose']),
                running_mode=RunningMode.LIVE_STREAM,
                min_pose_detection_confidence=self.min_det,
                min_pose_presence_confidence=self.min_pres,
                min_tracking_confidence=self.min_trk,
                result_callback=self._on_pose
            ))
            self.get_logger().info(f'Pose model: {self.paths["pose"]}')

        elif self.model == 'face':
            self.face = FaceLandmarker.create_from_options(FaceLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=self.paths['face']),
                running_mode=RunningMode.LIVE_STREAM,
                min_face_detection_confidence=self.min_det,
                min_face_presence_confidence=self.min_pres,
                min_tracking_confidence=self.min_trk,
                result_callback=self._on_face
            ))
            self.get_logger().info(f'Face model: {self.paths["face"]}')
        else:
            self.get_logger().error("param 'model' must be one of: hand|pose|face")

        self.get_logger().info(f"MP node ready: model={self.model}, topic_prefix={self.topic_prefix}")

    # -------- Image callback --------
    def on_image(self, msg: Image):
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
        ts_ms = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1e6) or int(time.time()*1000)

        if self.model == 'hand':
            if self.gesture is not None:
                self.gesture.recognize_async(mp_img, ts_ms)
            if self.hand is not None:
                self.hand.detect_async(mp_img, ts_ms)
        elif self.model == 'pose' and self.pose is not None:
            self.pose.detect_async(mp_img, ts_ms)
        elif self.model == 'face' and self.face is not None:
            self.face.detect_async(mp_img, ts_ms)

    # -------- Result callbacks --------
    def _on_gesture(self, result, output_image, timestamp_ms):
        if not result.gestures:
            return
        top = result.gestures[0][0]
        m = HandGesture()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.gesture = top.category_name or top.display_name
        m.score = float(top.score)
        self.pub_gesture.publish(m)

    def _on_hand(self, result, output_image, timestamp_ms):
        if not result.hand_landmarks:
            self._maybe_debug(self.last_bgr, [])
            return
        hlm = HandLandmarks()
        hlm.header = Header()
        hlm.header.stamp = self.get_clock().now().to_msg()
        hlm.header.frame_id = self.frame_id

        ma = MarkerArray(); mid = 0
        w, h = self.last_size

        for i, hand in enumerate(result.hand_landmarks):
            hmsg = Hand()
            # handedness
            if result.handedness and i < len(result.handedness) and result.handedness[i]:
                hd = result.handedness[i][0]
                hmsg.handedness = hd.category_name or (hd.display_name or '')
                hmsg.score = float(hd.score)
            else:
                hmsg.handedness = ''
                hmsg.score = 0.0

            # landmarks
            pts = []
            for lm in hand:
                x = float(lm.x); y = float(lm.y); z = float(lm.z)
                if self.coord_mode == 'pixel':
                    pts.append(Point(x=x*w, y=y*h, z=z))
                else:
                    pts.append(Point(x=x, y=y, z=z))
            hmsg.landmarks.extend(pts)
            hlm.hands.append(hmsg)

            # Markers: points + lines
            mpts = Marker()
            mpts.header.frame_id = self.frame_id
            mpts.header.stamp = hlm.header.stamp
            mpts.ns = 'hand_landmarks'; mpts.id = mid; mid += 1
            mpts.type = Marker.SPHERE_LIST; mpts.action = Marker.ADD
            mpts.scale.x = mpts.scale.y = mpts.scale.z = 0.02
            mpts.color.a = 1.0
            if hmsg.handedness.lower().startswith('left'): mpts.color.g = 1.0
            elif hmsg.handedness.lower().startswith('right'): mpts.color.r = 1.0
            else: mpts.color.b = 1.0
            mpts.points.extend(pts)
            ma.markers.append(mpts)

            lines = Marker()
            lines.header.frame_id = self.frame_id
            lines.header.stamp = hlm.header.stamp
            lines.ns = 'hand_skeleton'; lines.id = mid; mid += 1
            lines.type = Marker.LINE_LIST; lines.action = Marker.ADD
            lines.scale.x = 0.01
            lines.color.a = 1.0; lines.color.r = lines.color.g = lines.color.b = 1.0
            for a,b in HAND_EDGES:
                lines.points.append(pts[a]); lines.points.append(pts[b])
            ma.markers.append(lines)

        self.pub_hand_lm.publish(hlm)
        self.pub_markers.publish(ma)
        self._maybe_debug(self.last_bgr, result.hand_landmarks, result.handedness)

    def _on_pose(self, result, output_image, timestamp_ms):
        if not result.pose_landmarks:
            self._maybe_debug(self.last_bgr, [])
            return
        plm = PoseLandmarks()
        plm.header = Header(); plm.header.stamp = self.get_clock().now().to_msg(); plm.header.frame_id = self.frame_id
        w, h = self.last_size

        # Use the first person only (MediaPipe returns list)
        pose = result.pose_landmarks[0]
        pts = []
        for lm in pose:
            x = float(lm.x); y = float(lm.y); z = float(lm.z)
            if self.coord_mode == 'pixel':
                pts.append(Point(x=x*w, y=y*h, z=z))
            else:
                pts.append(Point(x=x, y=y, z=z))
        plm.landmarks.extend(pts)
        plm.score = 0.0
        self.pub_pose_lm.publish(plm)

        # Markers
        ma = MarkerArray(); mid = 0
        mpts = Marker()
        mpts.header.frame_id = self.frame_id
        mpts.header.stamp = plm.header.stamp
        mpts.ns = 'pose_landmarks'; mpts.id = mid; mid += 1
        mpts.type = Marker.SPHERE_LIST; mpts.action = Marker.ADD
        mpts.scale.x = mpts.scale.y = mpts.scale.z = 0.02
        mpts.color.a = 1.0; mpts.color.b = 1.0
        mpts.points.extend(pts)
        ma.markers.append(mpts)

        lines = Marker()
        lines.header.frame_id = self.frame_id
        lines.header.stamp = plm.header.stamp
        lines.ns = 'pose_skeleton'; lines.id = mid; mid += 1
        lines.type = Marker.LINE_LIST; lines.action = Marker.ADD
        lines.scale.x = 0.01
        lines.color.a = 1.0; lines.color.r = lines.color.g = lines.color.b = 1.0
        for a,b in POSE_EDGES:
            if a < len(pts) and b < len(pts):
                lines.points.append(pts[a]); lines.points.append(pts[b])
        ma.markers.append(lines)
        self.pub_markers.publish(ma)

        self._maybe_debug(self.last_bgr, [pose])

    def _on_face(self, result, output_image, timestamp_ms):
        if not result.face_landmarks:
            self._maybe_debug(self.last_bgr, [])
            return
        flm = FaceLandmarks()
        flm.header = Header(); flm.header.stamp = self.get_clock().now().to_msg(); flm.header.frame_id = self.frame_id
        w, h = self.last_size
        face = result.face_landmarks[0]
        pts = []
        for lm in face:
            x = float(lm.x); y = float(lm.y); z = float(lm.z)
            if self.coord_mode == 'pixel':
                pts.append(Point(x=x*w, y=y*h, z=z))
            else:
                pts.append(Point(x=x, y=y, z=z))
        flm.landmarks.extend(pts)
        flm.score = 0.0
        self.pub_face_lm.publish(flm)

        # Markers (points cloud only: 468 points)
        ma = MarkerArray(); mid = 0
        mpts = Marker()
        mpts.header.frame_id = self.frame_id
        mpts.header.stamp = flm.header.stamp
        mpts.ns = 'face_landmarks'; mpts.id = mid; mid += 1
        mpts.type = Marker.SPHERE_LIST; mpts.action = Marker.ADD
        mpts.scale.x = mpts.scale.y = mpts.scale.z = 0.01
        mpts.color.a = 1.0; mpts.color.g = 1.0
        mpts.points.extend(pts)
        ma.markers.append(mpts)
        self.pub_markers.publish(ma)

        self._maybe_debug(self.last_bgr, [face])

    # -------- Debug overlay publisher --------
    def _maybe_debug(self, bgr, landmarks_list, handedness_list=None):
        if not self.pub_dbg or bgr is None:
            return
        dbg = bgr.copy()
        w, h = self.last_size

        def to_px(lm):
            return (int(float(lm.x)*w), int(float(lm.y)*h))

        if self.model == 'hand':
            for i, hand in enumerate(landmarks_list):
                color = (0,255,0)
                if handedness_list and i < len(handedness_list) and handedness_list[i]:
                    name = (handedness_list[i][0].category_name or '').lower()
                    if name.startswith('right'): color = (0,0,255)
                    elif name.startswith('left'): color = (0,255,0)
                pts_px = [to_px(lm) for lm in hand]
                for p in pts_px: cv2.circle(dbg, p, 3, color, -1, lineType=cv2.LINE_AA)
                for a,b in HAND_EDGES: cv2.line(dbg, pts_px[a], pts_px[b], (255,255,255), 2, lineType=cv2.LINE_AA)
                xs = [p[0] for p in pts_px]; ys = [p[1] for p in pts_px]
                cv2.rectangle(dbg, (min(xs),min(ys)), (max(xs),max(ys)), color, 2)
        elif self.model == 'pose' and landmarks_list:
            pose = landmarks_list[0]
            pts_px = [to_px(lm) for lm in pose]
            for p in pts_px: cv2.circle(dbg, p, 2, (255,255,0), -1, lineType=cv2.LINE_AA)
            for a,b in POSE_EDGES:
                if a < len(pts_px) and b < len(pts_px):
                    cv2.line(dbg, pts_px[a], pts_px[b], (255,255,255), 2, lineType=cv2.LINE_AA)
        elif self.model == 'face' and landmarks_list:
            face = landmarks_list[0]
            pts_px = [to_px(lm) for lm in face]
            for p in pts_px: cv2.circle(dbg, p, 1, (0,255,255), -1, lineType=cv2.LINE_AA)
            xs = [p[0] for p in pts_px]; ys = [p[1] for p in pts_px]
            cv2.rectangle(dbg, (min(xs),min(ys)), (max(xs),max(ys)), (0,255,255), 2)

        img_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.frame_id
        self.pub_debug.publish(img_msg)

def main():
    rclpy.init()
    rclpy.spin(MPNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()