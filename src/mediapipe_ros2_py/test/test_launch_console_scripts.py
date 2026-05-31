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

"""Static launch/console-script contract tests that do not require hardware."""

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PY_PACKAGE_DIR = REPO_ROOT / 'src' / 'mediapipe_ros2_py'
PY_MODULE_DIR = PY_PACKAGE_DIR / 'mediapipe_ros2_py'
NODE_PACKAGE_DIR = REPO_ROOT / 'src' / 'mediapipe_ros2_node'

HAND_DEMO_LAUNCHES = {
    Path('src/mediapipe_ros2_node/launch/hand_demo.launch.py'),
    Path('src/mediapipe_ros2_node/launch/full_demo.launch.py'),
    Path('src/mediapipe_ros2_node/launch/viz_demo.launch.py'),
}
REQUIRED_HAND_DEMO_PARAMS = {
    'model',
    'image_topic',
    'topic_prefix',
    'use_gesture',
    'publish_debug_image',
}
LEGACY_ONLY_PARAMS = {'use_landmarks'}


def _read_tree(path):
    return ast.parse(path.read_text(), filename=str(path))


def _keyword_value(call, keyword_name):
    for keyword in call.keywords:
        if keyword.arg == keyword_name:
            return keyword.value
    return None


def _keyword_string(call, keyword_name):
    value = _keyword_value(call, keyword_name)
    if isinstance(value, ast.Constant) and isinstance(value.value, str):
        return value.value
    return None


def _console_script_entries():
    setup_tree = _read_tree(PY_PACKAGE_DIR / 'setup.py')
    for call in (node for node in ast.walk(setup_tree) if isinstance(node, ast.Call)):
        entry_points = _keyword_value(call, 'entry_points')
        if not isinstance(entry_points, ast.Dict):
            continue
        for key_node, value_node in zip(entry_points.keys, entry_points.values):
            if not isinstance(key_node, ast.Constant) or key_node.value != 'console_scripts':
                continue
            entries = {}
            for entry in value_node.elts:
                if not isinstance(entry, ast.Constant) or not isinstance(entry.value, str):
                    continue
                name, target = entry.value.split('=', 1)
                entries[name.strip()] = target.strip()
            return entries
    return {}


def _mediapipe_py_launch_nodes():
    launch_dir = NODE_PACKAGE_DIR / 'launch'
    for launch_file in sorted(launch_dir.glob('*.launch.py')):
        launch_tree = _read_tree(launch_file)
        for call in (node for node in ast.walk(launch_tree) if isinstance(node, ast.Call)):
            if not isinstance(call.func, ast.Name) or call.func.id != 'Node':
                continue
            package = _keyword_string(call, 'package')
            executable = _keyword_string(call, 'executable')
            if package == 'mediapipe_ros2_py' and executable:
                yield launch_file.relative_to(REPO_ROOT), call


def _parameter_keys(node_call):
    parameters = _keyword_value(node_call, 'parameters')
    keys = set()
    if not isinstance(parameters, ast.List):
        return keys
    for item in parameters.elts:
        if not isinstance(item, ast.Dict):
            continue
        for key_node in item.keys:
            if isinstance(key_node, ast.Constant) and isinstance(key_node.value, str):
                keys.add(key_node.value)
    return keys


def _parameter_constant(node_call, parameter_name):
    parameters = _keyword_value(node_call, 'parameters')
    if not isinstance(parameters, ast.List):
        return None
    for item in parameters.elts:
        if not isinstance(item, ast.Dict):
            continue
        for key_node, value_node in zip(item.keys, item.values):
            if not isinstance(key_node, ast.Constant) or key_node.value != parameter_name:
                continue
            if isinstance(value_node, ast.Constant):
                return value_node.value
    return None


def _declared_parameter_names(module_path):
    tree = _read_tree(module_path)
    names = set()
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if not isinstance(call.func, ast.Attribute):
            continue
        if call.func.attr != 'declare_parameter' or not call.args:
            continue
        first_arg = call.args[0]
        if isinstance(first_arg, ast.Constant) and isinstance(first_arg.value, str):
            names.add(first_arg.value)
    return names


def _defined_function_names(module_path):
    tree = _read_tree(module_path)
    return {node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}


def test_mediapipe_py_launch_executables_are_console_scripts():
    console_scripts = _console_script_entries()
    assert console_scripts, 'No console_scripts found in mediapipe_ros2_py/setup.py'

    missing = [
        f'{launch_file}: {_keyword_string(node_call, "executable")}'
        for launch_file, node_call in _mediapipe_py_launch_nodes()
        if _keyword_string(node_call, 'executable') not in console_scripts
    ]
    assert not missing, (
        'Launch files reference executables that are not registered as '
        f'mediapipe_ros2_py console_scripts: {missing}'
    )


def test_console_script_targets_have_python_modules_and_main_functions():
    missing = []
    for executable, target in _console_script_entries().items():
        module_name, function_name = target.split(':', 1)
        module_path = PY_PACKAGE_DIR / Path(*module_name.split('.')).with_suffix('.py')
        if not module_path.exists():
            missing.append(f'{executable}: missing module {module_path.relative_to(REPO_ROOT)}')
            continue
        if function_name not in _defined_function_names(module_path):
            missing.append(
                f'{executable}: missing function {function_name} in '
                f'{module_path.relative_to(REPO_ROOT)}'
            )
    assert not missing


def test_hand_demo_launches_use_unified_mp_node_executable():
    wrong_executables = []
    seen_launches = set()
    for launch_file, node_call in _mediapipe_py_launch_nodes():
        if launch_file not in HAND_DEMO_LAUNCHES:
            continue
        seen_launches.add(launch_file)
        executable = _keyword_string(node_call, 'executable')
        if executable != 'mp_node':
            wrong_executables.append(f'{launch_file}: {executable}')

    assert seen_launches == HAND_DEMO_LAUNCHES
    assert not wrong_executables, (
        'Hand demo launch files must use the unified mp_node executable, '
        f'not legacy hand_node: {wrong_executables}'
    )


def test_hand_demo_launch_parameters_match_mp_node_contract():
    mp_node_params = _declared_parameter_names(PY_MODULE_DIR / 'mp_node.py')
    assert REQUIRED_HAND_DEMO_PARAMS <= mp_node_params

    failures = []
    for launch_file, node_call in _mediapipe_py_launch_nodes():
        if launch_file not in HAND_DEMO_LAUNCHES:
            continue
        launch_params = _parameter_keys(node_call)
        missing_required = REQUIRED_HAND_DEMO_PARAMS - launch_params
        unsupported = launch_params - mp_node_params
        legacy_only = launch_params & LEGACY_ONLY_PARAMS
        model = _parameter_constant(node_call, 'model')
        if missing_required:
            failures.append(f'{launch_file}: missing {sorted(missing_required)}')
        if unsupported:
            failures.append(f'{launch_file}: unsupported by mp_node.py {sorted(unsupported)}')
        if legacy_only:
            failures.append(f'{launch_file}: legacy-only params {sorted(legacy_only)}')
        if model != 'hand':
            failures.append(f'{launch_file}: model must be literal "hand", got {model!r}')

    assert not failures


def test_legacy_hand_node_is_a_compatibility_console_script():
    console_scripts = _console_script_entries()
    assert console_scripts.get('hand_node') == (
        'mediapipe_ros2_py.hand_node_legacy:main'
    )
    legacy_path = PY_MODULE_DIR / 'hand_node_legacy.py'
    assert legacy_path.exists()
    legacy_source = legacy_path.read_text()
    assert 'from mediapipe_ros2_py.mp_node import main as mp_node_main' in legacy_source


def test_no_legacy_message_package_imports_remain():
    offenders = []
    for source_file in sorted((REPO_ROOT / 'src').rglob('*.py')):
        if 'third_party' in source_file.parts:
            continue
        source = source_file.read_text()
        legacy_import = 'mediapipe_ros2_node' + '.msg'
        if legacy_import in source:
            offenders.append(str(source_file.relative_to(REPO_ROOT)))
    assert not offenders
