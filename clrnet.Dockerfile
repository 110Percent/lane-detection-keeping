# This Dockerfile is used to build the CLRNet environment on which the Detection subsystem will run.
# CLRNet is a deep learning model for lane detection. It provides a Python API to use the model and visualize its results.

FROM nvidia/cuda:11.6.1-cudnn8-devel-ubuntu20.04 as runtime

ARG DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update
RUN apt-get install -y build-essential git ninja-build python3-pip wget unzip

RUN python3 -m pip install torch==1.8.2 torchvision==0.9.2 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cu111

# Clone CLRNet from its GitHub repository
RUN mkdir -p /opt/clrnet
RUN git clone https://github.com/Turoad/CLRNet.git /opt/clrnet
WORKDIR /opt/clrnet

# CLRNet's requirements.txt file is not compatible with the latest versions of the libraries it uses.
# We need to modify it to use the versions that are compatible with the current version of PyTorch,
# along with fixing some package names.
RUN sed -i '/torch/d' requirements.txt
RUN sed -i '/torchvision/d' requirements.txt
RUN sed -i '1s/^/numpy==1.23.1\n/' requirements.txt
RUN sed -i 's|sklearn|scikit-learn|g' requirements.txt

# Install CLRNet's dependencies
RUN python3 -m pip install -r requirements.txt

# Build and install CLRNet
RUN python3 setup.py build develop

# Patch the visualization script to avoid hanging the process when displaying images.
# This change causes the OpenCV window to update every 10ms instead of waiting for a key press.
RUN sed -i "s|cv2.waitKey(0)|cv2.waitKey(10)|g" /opt/clrnet/clrnet/utils/visualization.py

# Download the pre-trained TuSimple model
RUN mkdir -p /opt/clrnet/models
WORKDIR /opt/clrnet/models
RUN wget -q 'https://github.com/Turoad/CLRNet/releases/download/models/tusimple_r18.pth.zip'
RUN unzip tusimple_r18.pth.zip
RUN rm tusimple_r18.pth.zip

# Download the pre-trained ResNet18 model
RUN mkdir -p /root/.cache/torch/hub/checkpoints
RUN wget -q -O /root/.cache/torch/hub/checkpoints/resnet18-5c106cde.pth https://download.pytorch.org/models/resnet18-5c106cde.pth

WORKDIR /opt/clrnet