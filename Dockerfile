FROM ros:foxy-ros-base

# ENV HTTP_PROXY=  # 不需要代理
# ENV HTTPS_PROXY=
ENV NO_PROXY=localhost,127.0.0.1,192.168.31.0/24
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# apt 清华镜像
RUN sed -i 's|ports.ubuntu.com|mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' /etc/apt/sources.list && \
    apt-get update

# CycloneDDS + SHM禁用 (根治容器TF问题)
ENV FASTRTPS_SHM_DISABLE=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 系统 + ROS2 依赖
RUN apt-get install -y --no-install-recommends \
    python3-pip python3-dev python3-numpy python3-opencv \
    python3-serial python3-flask python3-pil python3-requests \
    v4l-utils usbutils i2c-tools \
    ros-foxy-tf2-ros ros-foxy-tf2-tools ros-foxy-cv-bridge \
    ros-foxy-image-transport ros-foxy-vision-opencv \
    ros-foxy-vision-msgs ros-foxy-nav2-msgs \
    ros-foxy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Python 依赖
RUN pip3 install --no-cache-dir pyserial smbus2 onnxruntime

# Rosmaster_Lib
COPY Rosmaster_Lib.py /usr/local/lib/python3.8/dist-packages/Rosmaster_Lib.py

RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

WORKDIR /home/pi/patrol_robot/patrol_robot


