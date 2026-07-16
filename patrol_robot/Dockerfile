FROM yahboomtechnology/ros-foxy:4.0.9R2

# 安装 ROS2 消息包
RUN apt-get update -qq && \
    apt-get install -y -qq ros-foxy-vision-msgs && \
    rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖 (清华镜像加速)
RUN pip3 install --user --no-cache-dir \
      -i https://pypi.tuna.tsinghua.edu.cn/simple \
      onnxruntime flask

# 设置 ROS2 环境
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

WORKDIR /home/pi/patrol_robot/patrol_robot
