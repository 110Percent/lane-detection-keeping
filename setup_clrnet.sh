#!/bin/bash

echo "Activating Python virtual environment"
source ./venv/bin/activate

echo "Installing PyTorch dependencies"
python -m pip install torch==1.8.2 torchvision==0.9.2 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cu111

if [ -d "ros2_ws/src/lane_nodes_py/lane_nodes_py/clrnet" ]; then
  echo "CLRNet is already installed in the ROS2 workspace"
  exit 0
fi

if [ -d "CLRNet" ]; then
  echo "CLRNet already cloned"
else
  echo "Cloning CLRNet repository"
  git clone https://github.com/Turoad/CLRNet.git
fi

cd CLRNet || exit

if ! grep -q numpy "requirements.txt"; then
  echo "Fixing dependencies"
  sed -i '/torch/d' requirements.txt
  sed -i '/torchvision/d' requirements.txt
  sed -i '1s/^/numpy==1.19\n/' requirements.txt
  sed -i 's|sklearn|scikit-learn|g' requirements.txt

  echo "Installing CLRNet dependencies"
  python -m pip install -r requirements.txt
fi

if [ -d "build" ]; then
  echo "CLRNet already built"
else
  echo "Building CLRNet"
  python setup.py build develop
fi

echo "Copying CLRNet to ROS2 workspace"
cp -r clrnet ../ros2_ws/src/lane_nodes_py/lane_nodes_py
