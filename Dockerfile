FROM osrf/ros:humble-desktop-full

# --- Dependencias ROS adicionales ---
RUN apt-get update && apt-get install -y \
    wget \
    bzip2 \
    git \
    curl \
    gnupg \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# --- Instalar Gazebo Classic y plugins ROS2 ---
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher \
    ros-humble-control-msgs \
    ros-humble-irobot-create-msgs \
    ros-humble-ign-ros2-control \
    ros-humble-joint-state-broadcaster \
    ros-humble-ros2-controllers \
    ros-humble-rsl \
    ros-humble-gazebo-dev \
    ros-humble-nav2-rviz-plugins \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# sudo apt install ros-humble-slam-toolbox

# --- Modelos de Gazebo ---
RUN git clone https://github.com/osrf/gazebo_models.git /usr/share/gazebo/models

# --- Dependencias Python del sistema ---
RUN pip3 install --no-cache-dir \
    "numpy<2" \
    pandas \
    pyqtgraph==0.12.4 \
    PyQt5==5.14.1 \
    lxml \
    pipreqs \
    setuptools==59.6.0

# --- PyTorch con CUDA 11.3 ---
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu113

# --- Variables de entorno ROS/Gazebo ---
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo/models

# ============================================================
# === Instalar SLAM Toolbox Lifecycle desde el repositorio ===
# ============================================================

# Crear workspace
#RUN mkdir -p /ros2_ws/src
#WORKDIR /ros2_ws/src

# Clonar la versión lifecycle
#RUN git clone -b humble_lifecycle https://github.com/SteveMacenski/slam_toolbox.git

# Instalar dependencias
#WORKDIR /ros2_ws
#RUN apt-get update && rosdep update && \
#    rosdep install -y --from-paths src --ignore-src && \
#    rm -rf /var/lib/apt/lists/*

# Compilar
#RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Añadir overlay permanente
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc 
#&& \
#    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc

# ============================================================

# --- Copiar scripts de ROS ---
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
