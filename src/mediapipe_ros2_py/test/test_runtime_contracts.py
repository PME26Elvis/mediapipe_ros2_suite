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

"""Hardware-free runtime contract tests for the MediaPipe ROS 2 node."""

import ast
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
PY_PACKAGE_DIR = REPO_ROOT / 'src' / 'mediapipe_ros2_py'
sys.path.insert(0, str(PY_PACKAGE_DIR))

from mediapipe_ros2_py.runtime_contracts import (  # noqa: E402
    HAND_EDGES,
    HAND_LANDMARK_COUNT,
    MODEL_FILENAMES,
    POSE_EDGES,
    POSE_LANDMARK_COUNT,
    TOPIC_SUFFIXES,
    landmark_xyz,
    landmarks_xyz,
    model_asset_path,
    model_key_for,
    timestamp_ms_from_stamp,
    topic_name,
    top_category_label_score,
)


def _lm(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def _stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_model_asset_contract_matches_expected_task_filenames():
    assert MODEL_FILENAMES == {
        'hand': 'hand_landmarker.task',
        'gesture': 'gesture_recognizer.task',
        'pose': 'pose_landmarker.task',
        'face': 'face_landmarker.task',
    }
    assert model_key_for('hand') == 'hand'
    assert model_key_for('gesture') == 'gesture'
    assert model_key_for('unknown') == 'hand'
    assert str(model_asset_path('/tmp/models', 'pose')).endswith(
        '/tmp/models/pose_landmarker.task'
    )


def test_topic_contract_uses_canonical_prefix_and_suffixes():
    assert TOPIC_SUFFIXES['markers'] == '/markers'
    assert topic_name('/mediapipe/', 'hand_landmarks') == '/mediapipe/hand/landmarks'
    assert topic_name('/mediapipe', 'hand_gesture') == '/mediapipe/hand/gesture'
    assert topic_name('/mediapipe', 'pose_landmarks') == '/mediapipe/pose/landmarks'
    assert topic_name('/mediapipe', 'face_landmarks') == '/mediapipe/face/landmarks'


def test_timestamp_contract_uses_header_stamp_unless_it_is_zero():
    assert timestamp_ms_from_stamp(_stamp(12, 345_000_000), 9999) == 12345
    assert timestamp_ms_from_stamp(_stamp(0, 0), 4242) == 4242


def test_landmark_coordinate_contract_supports_normalized_and_pixel_modes():
    landmark = _lm(0.25, 0.5, -0.125)
    assert landmark_xyz(landmark, (640, 480), 'normalized') == (0.25, 0.5, -0.125)
    assert landmark_xyz(landmark, (640, 480), 'pixel') == (160.0, 240.0, -0.125)
    assert landmarks_xyz([landmark], (640, 480), 'pixel') == [(160.0, 240.0, -0.125)]
    with pytest.raises(ValueError, match='coord_mode'):
        landmark_xyz(landmark, (640, 480), 'meters')


def test_category_contract_prefers_category_name_then_display_name():
    named = SimpleNamespace(category_name='Closed_Fist', display_name='fist', score=0.91)
    displayed = SimpleNamespace(category_name='', display_name='Palm', score=0.72)
    assert top_category_label_score([named]) == ('Closed_Fist', 0.91)
    assert top_category_label_score([displayed]) == ('Palm', 0.72)
    assert top_category_label_score([]) == ('', 0.0)


def test_skeleton_edges_stay_inside_known_mediapipe_landmark_ranges():
    assert max(max(edge) for edge in HAND_EDGES) < HAND_LANDMARK_COUNT
    assert min(min(edge) for edge in HAND_EDGES) >= 0
    assert max(max(edge) for edge in POSE_EDGES) < POSE_LANDMARK_COUNT
    assert min(min(edge) for edge in POSE_EDGES) >= 0


def test_mp_node_is_wired_to_hardware_free_runtime_contract_helpers():
    tree = ast.parse((PY_PACKAGE_DIR / 'mediapipe_ros2_py' / 'mp_node.py').read_text())
    imported_names = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
        and node.module == 'mediapipe_ros2_py.runtime_contracts'
        for alias in node.names
    }
    calls = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }

    assert {
        'HAND_EDGES',
        'POSE_EDGES',
        'MODEL_FILENAMES',
        'landmarks_xyz',
        'model_asset_path',
        'timestamp_ms_from_stamp',
        'topic_name',
        'top_category_label_score',
    } <= imported_names
    assert {
        'landmarks_xyz',
        'model_asset_path',
        'timestamp_ms_from_stamp',
        'topic_name',
        'top_category_label_score',
    } <= calls
