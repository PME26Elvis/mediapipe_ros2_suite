# Contributing to ROS 2 MediaPipe Suite

Thanks for your interest in contributing! This project turns MediaPipe Tasks into reusable ROS 2 components with clean message contracts and reproducible launches. Contributions that improve **usability, maintainability, or reproducibility** are especially welcome.

## Getting started

- Target ROS 2: **Humble** (CI) and **Jazzy** (experimental CI).
- Workspace:
  ```bash
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions
  cd ~/ros2_ws
  # clone into src if not already
  # git clone [https://github.com/PME26Elvis/mediapipe_ros2_suite](https://github.com/PME26Elvis/mediapipe_ros2_suite) src/mediapipe_ros2_suite
  colcon build --symlink-install
  source install/setup.bash
  ````
- Models: this repo does not redistribute .task assets. Place them under `src/mediapipe_ros2_node/models/` using the expected filenames: `hand_landmarker.task`, `gesture_recognizer.task`, `pose_landmarker.task`, `face_landmarker.task`. You may also set `MP_MODELS_DIR` to point to a custom folder.

## Running locally

```bash
# camera
ros2 run v4l2_camera v4l2_camera_node

# node (parameterized)
ros2 launch mediapipe_ros2_py mp_node.launch.py model:=hand image_topic:=/image_raw start_rviz:=true
```

**Example: gesture → turtlesim**

```bash
ros2 run turtlesim turtlesim_node
ros2 run mediapipe_ros2_py gesture_to_turtlesim
```

**Optional: multi-camera example**

```bash
ros2 launch mediapipe_ros2_py two_cams.launch.py
```

## Issue reports

When filing an issue, please include:

  - ROS 2 distro, OS version, Python version
  - Exact command(s) you ran and full log (or minimal repro)
  - Whether models are in the expected path or via `MP_MODELS_DIR`

## Pull requests

Scope small, ship fast. Prefer focused PRs (one logical change).

Checklist:

  - Builds on Humble locally (`colcon build`)
  - Passes CI (Actions) or at least doesn’t break Humble
  - No model binaries added to git
  - Updated README/launch/docs if behavior or parameters change
  - Clear commit message and PR description

## Coding style

  - Python: follow PEP 8 (`black`/`ruff` welcome but not required).
  - ROS: keep message contracts stable; if you must change them, propose first.
  - Logging: prefer concise INFO lines; keep FPS log to \~1 line/sec.

## Branches & versioning

  - Branch from `main`, open PRs against `main`.
  - Tags: `vMAJOR.MINOR.PATCH` (e.g., `v0.1.1`).
  - Breaking message/interface changes bump MINOR at least.

## License

By contributing, you agree your contributions are licensed under Apache-2.0. Do not commit third-party model files; respect upstream licenses.

````

---

### CODE_OF_CONDUCT.md

```markdown
# Code of Conduct

We are committed to a welcoming, harassment-free community for everyone.

## Our Standards
- Be respectful and professional in all interactions.
- Assume good intent; prefer clarification over accusation.
- No harassment, hate speech, or personal attacks.
- Keep feedback technical, actionable, and kind.

## Scope
This Code applies within project spaces (issues, PRs, discussions) and in public spaces when an individual is representing the project.

## Enforcement
Report unacceptable behavior to the maintainers at:
elvisyen0727@gmail.com

Maintainers may take any action deemed appropriate, including warnings or temporary/permanent bans from participation.

## Attribution
This policy is adapted from common open-source codes of conduct and the spirit of the Contributor Covenant.
````
