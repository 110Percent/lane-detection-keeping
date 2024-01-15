FROM nvidia/cuda:11.6.1-cudnn8-devel-ubuntu20.04 as runtime

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y build-essential git ninja-build python3-pip wget unzip

RUN python3 -m pip install torch==1.8.2 torchvision==0.9.2 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cu111

RUN mkdir -p /opt/clrnet
RUN git clone https://github.com/Turoad/CLRNet.git /opt/clrnet
WORKDIR /opt/clrnet

RUN sed -i '/torch/d' requirements.txt
RUN sed -i '/torchvision/d' requirements.txt
RUN sed -i '1s/^/numpy==1.23.1\n/' requirements.txt
RUN sed -i 's|sklearn|scikit-learn|g' requirements.txt

RUN python3 -m pip install -r requirements.txt

RUN python3 setup.py build develop

RUN mkdir -p /opt/clrnet/models
WORKDIR /opt/clrnet/models
RUN wget -q 'https://github.com/Turoad/CLRNet/releases/download/models/tusimple_r18.pth.zip'
RUN unzip tusimple_r18.pth.zip
RUN rm tusimple_r18.pth.zip

WORKDIR /opt/clrnet