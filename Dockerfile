FROM osrf/ros:humble-desktop-full

# --- Dependencias ROS adicionales ---
RUN apt-get update && apt-get install -y \
    wget \
    bzip2 \
    git \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# --- Instalar Gazebo Classic y ROS2 plugins ---
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
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3 \
    python3-pip \
    python3-lxml \
    && rm -rf /var/lib/apt/lists/*

RUN pip install lxml numpy

    # --- Modelos de Gazebo ---
RUN git clone https://github.com/osrf/gazebo_models.git /usr/share/gazebo/models

# --- Instalar pipreqs y dependencias Python repo create3 ---
RUN pip install --no-cache-dir \
    setuptools==59.6.0 \
    pipreqs

# --- CUDA runtime 11.3 (para PyTorch 1.10.0+cu113) ---
# Aunque el host tenga CUDA 12.x, PyTorch trae sus propias librerías cu113.
# Solo necesitamos un driver moderno en el host.
ENV PATH=/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/sbin:/bin:/opt/ros/humble/bin:/usr/local/cuda-11.3/bin:/opt/conda/bin:/opt/conda/condabin
ENV PATH=/usr/local/cuda-11.3/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-11.3/lib64

# --- Instalar Miniconda ---
ENV CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p $CONDA_DIR && \
    rm /tmp/miniconda.sh && \
    $CONDA_DIR/bin/conda clean -afy

# Añadir conda al PATH
ENV PATH=$CONDA_DIR/bin:$PATH

# --- Copiar el environment.yml ---
COPY environment.yml /tmp/environment.yml

# --- Crear entorno conda turtle3-drlnav ---
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

RUN conda env create -f /tmp/environment.yml && conda clean -afy

# Activar entorno por defecto en bash
RUN echo "source activate turtle3-drlnav" >> ~/.bashrc
#RUN echo "source /home/$USER/turtlebot3_ws/setup_drlnav.sh" >> ~/.bashrc
#RUN echo "source /home/$USER/create3_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export PATH=/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/sbin:/bin:/opt/ros/humble/bin:/usr/local/cuda-11.3/bin:/opt/conda/bin:/opt/conda/condabin" >> ~/.bashrc

#ENV PATH=/opt/conda/envs/turtle3-drlnav/bin:${PATH}


# --- Copiar scripts de ROS ---
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# --- Variables de entorno ROS/Gazebo ---
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo/models

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
