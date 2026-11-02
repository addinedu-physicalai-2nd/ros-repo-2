#!/usr/bin/env bash
# 로봇에 안내(guide_node)용 YOLO venv 만들기: ~/venvs/yolo 에 ultralytics + CPU torch
#   PC에서 로봇별로:  ssh pinky1 'bash -s' < ros_ws/src/fleet_agent/tools/setup_yolo_venv.sh
#   - --system-site-packages: venv 안에서도 rclpy 등 ROS 패키지가 보이게
#   - PYTHONNOUSERSITE=1: ~/.local에 깔린 것을 "이미 있음"으로 보지 않고 venv 안에 진짜로 설치
#   - numpy<2, opencv-python<4.12: numpy 2가 딸려 와 ROS(numpy 1.x 빌드)와 충돌하지 않게
#   - launch(fleet_agent.launch.xml)가 guide 노드에만 이 venv를 PYTHONPATH 앞에 붙인다
# 마지막 줄이 모델 클래스(toy_person ...)를 찍으면 끝. 모델이 없으면 여기서 멈춤 (models/best.pt 복사 후 빌드)
set -e
V=$HOME/venvs/yolo
M=$HOME/ros-repo-2/ros_ws/install/fleet_agent/share/fleet_agent/models/best.pt
export PYTHONNOUSERSITE=1
echo "== $(hostname) ($(uname -m))"
rm -rf "$V"
python3 -m venv --system-site-packages "$V"
"$V/bin/pip" install -q --upgrade pip
"$V/bin/pip" install -q torch torchvision --index-url https://download.pytorch.org/whl/cpu
"$V/bin/pip" install -q ultralytics "numpy<2" "opencv-python<4.12"
"$V/bin/python" -c "import ultralytics, torch, numpy, cv2; print('OK', ultralytics.__version__, torch.__version__, 'numpy', numpy.__version__, 'cv2', cv2.__version__)"
"$V/bin/python" -c "from ultralytics import YOLO; print('model', YOLO('$M').names)"
