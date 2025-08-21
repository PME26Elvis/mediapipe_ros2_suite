FROM ros:humble
SHELL ["/bin/bash", "-lc"]

# 基本工具與相機/橋接
RUN apt-get update && apt-get install -y \
    python3-pip ros-humble-v4l2-camera ros-humble-cv-bridge ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# mediapipe + opencv
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && pip install mediapipe opencv-python

# 建置工作區
WORKDIR /ws
COPY ./ /ws/src/
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install

# 預設指令（可改）
CMD . /opt/ros/humble/setup.bash && . /ws/install/setup.bash && \
    ros2 launch mediapipe_ros2_node viz_demo.launch.py
