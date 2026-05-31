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

"""Package metadata tests that do not require hardware or ROS daemons."""

import ast
from pathlib import Path
from xml.etree import ElementTree


REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_DIR = REPO_ROOT / 'src'
PLACEHOLDER_MAINTAINERS = {'you@example.com', 'you', 'You'}
EXPECTED_MAINTAINER_EMAIL = (
    'mediapipe-ros2-suite-maintainers@users.noreply.github.com'
)


def _package_xml(package_name):
    return SRC_DIR / package_name / 'package.xml'


def _package_root(package_name):
    return ElementTree.parse(_package_xml(package_name)).getroot()


def _tag_values(root, tag):
    return [element.text for element in root.findall(tag)]


def _setup_keyword(keyword_name):
    tree = ast.parse((SRC_DIR / 'mediapipe_ros2_py' / 'setup.py').read_text())
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        for keyword in call.keywords:
            if keyword.arg != keyword_name:
                continue
            value = keyword.value
            if isinstance(value, ast.Constant):
                return value.value
            if isinstance(value, ast.List):
                return [item.value for item in value.elts]
    raise AssertionError(f'{keyword_name} not found in setup.py')


def test_all_packages_have_non_placeholder_maintainers():
    failures = []
    for package_xml in sorted(SRC_DIR.glob('*/package.xml')):
        root = ElementTree.parse(package_xml).getroot()
        maintainer = root.find('maintainer')
        if maintainer is None:
            failures.append(f'{package_xml}: missing maintainer')
            continue
        email = maintainer.attrib.get('email', '')
        name = maintainer.text or ''
        if email in PLACEHOLDER_MAINTAINERS or name in PLACEHOLDER_MAINTAINERS:
            failures.append(f'{package_xml}: placeholder maintainer {name} <{email}>')
        if email != EXPECTED_MAINTAINER_EMAIL:
            failures.append(f'{package_xml}: unexpected maintainer email {email}')
    assert not failures


def test_python_package_declares_runtime_and_test_dependencies():
    root = _package_root('mediapipe_ros2_py')
    exec_depends = set(_tag_values(root, 'exec_depend'))
    test_depends = set(_tag_values(root, 'test_depend'))

    assert {
        'mediapipe_ros2_interfaces',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'visualization_msgs',
        'std_msgs',
        'cv_bridge',
        'ament_index_python',
        'python3-numpy',
        'python3-opencv',
    } <= exec_depends
    assert {
        'ament_copyright',
        'ament_flake8',
        'ament_pep257',
        'python3-pytest',
    } <= test_depends


def test_asset_package_declares_launch_runtime_dependencies():
    root = _package_root('mediapipe_ros2_node')
    exec_depends = set(_tag_values(root, 'exec_depend'))
    assert {'launch', 'launch_ros', 'mediapipe_ros2_py', 'rviz2', 'v4l2_camera'} <= exec_depends


def test_interface_package_declares_rosidl_membership_and_lint_dependencies():
    root = _package_root('mediapipe_ros2_interfaces')
    assert 'rosidl_interface_packages' in _tag_values(root, 'member_of_group')
    assert {'ament_lint_auto', 'ament_lint_common'} <= set(_tag_values(root, 'test_depend'))


def test_setup_py_metadata_matches_package_policy():
    assert _setup_keyword('maintainer') == 'MediaPipe ROS 2 Suite Maintainers'
    assert _setup_keyword('maintainer_email') == EXPECTED_MAINTAINER_EMAIL
    assert {'setuptools', 'numpy'} <= set(_setup_keyword('install_requires'))
