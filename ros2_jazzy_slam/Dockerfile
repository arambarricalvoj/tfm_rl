FROM osrf/ros:jazzy-desktop-full

# ============================
# Dependencias básicas
# ============================
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    wget \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# ============================
# Crear workspace
# ============================
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# ============================
# Clonar SLAM Toolbox (branch jazzy)
# ============================
RUN git clone -b jazzy https://github.com/SteveMacenski/slam_toolbox.git

# ============================
# Instalar dependencias ROS
# ============================
WORKDIR /ros2_ws
RUN apt-get update && rosdep update && \
    rosdep install -y --from-paths src --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# ============================
# Compilar SLAM Toolbox
# ============================
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# ============================
# Añadir overlay permanente
# ============================
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc

# ============================
# Entrypoint
# ============================
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
