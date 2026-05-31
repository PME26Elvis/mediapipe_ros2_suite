# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure runtime contracts for MediaPipe ROS 2 nodes.

This module intentionally has no ROS, OpenCV, or MediaPipe imports so CI can
exercise node semantics without a camera, model asset, or ROS daemon.
"""

from pathlib import Path

SUPPORTED_MODELS = ('hand', 'pose', 'face')
MODEL_FILENAMES = {
    'hand': 'hand_landmarker.task',
    'gesture': 'gesture_recognizer.task',
    'pose': 'pose_landmarker.task',
    'face': 'face_landmarker.task',
}
TOPIC_SUFFIXES = {
    'markers': '/markers',
    'debug_image': '/debug_image',
    'hand_landmarks': '/hand/landmarks',
    'hand_gesture': '/hand/gesture',
    'pose_landmarks': '/pose/landmarks',
    'face_landmarks': '/face/landmarks',
}
HAND_LANDMARK_COUNT = 21
POSE_LANDMARK_COUNT = 33
HAND_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
)
POSE_EDGES = (
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (23, 24), (11, 23), (12, 24), (23, 25), (25, 27),
    (24, 26), (26, 28),
)
VALID_COORD_MODES = ('normalized', 'pixel')


def normalize_topic_prefix(topic_prefix):
    """Return a canonical topic prefix without a trailing slash."""
    stripped = str(topic_prefix).rstrip('/')
    return stripped or ''


def model_key_for(model):
    """Return the asset lookup key used by the node for a requested model."""
    model_name = str(model).lower()
    if model_name in MODEL_FILENAMES:
        return model_name
    return 'hand'


def model_asset_path(models_dir, model):
    """Return the expected model asset path for a requested model."""
    return Path(models_dir) / MODEL_FILENAMES[model_key_for(model)]


def topic_name(topic_prefix, suffix_key):
    """Build a published topic name from a canonical suffix key."""
    return f'{normalize_topic_prefix(topic_prefix)}{TOPIC_SUFFIXES[suffix_key]}'


def timestamp_ms_from_stamp(stamp, fallback_ms):
    """Convert a ROS-like stamp into MediaPipe milliseconds.

    The node uses the provided fallback only when the incoming stamp is zero,
    matching bag/camera sources that omit header timestamps.
    """
    stamp_ms = int(stamp.sec * 1000 + stamp.nanosec / 1e6)
    return stamp_ms or int(fallback_ms)


def landmark_xyz(landmark, image_size, coord_mode):
    """Convert a MediaPipe-like landmark to normalized or pixel xyz tuple."""
    if coord_mode not in VALID_COORD_MODES:
        raise ValueError(
            f'coord_mode must be one of {VALID_COORD_MODES}, got {coord_mode!r}'
        )
    x = float(landmark.x)
    y = float(landmark.y)
    z = float(landmark.z)
    if coord_mode == 'pixel':
        width, height = image_size
        return x * width, y * height, z
    return x, y, z


def landmarks_xyz(landmarks, image_size, coord_mode):
    """Convert a sequence of MediaPipe-like landmarks to xyz tuples."""
    return [landmark_xyz(lm, image_size, coord_mode) for lm in landmarks]


def top_category_label_score(category_groups):
    """Return the top MediaPipe category label and score from nested groups."""
    if not category_groups or not category_groups[0]:
        return '', 0.0
    top = category_groups[0]
    return top.category_name or (top.display_name or ''), float(top.score)
