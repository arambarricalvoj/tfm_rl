FROM osrf/ros:humble-desktop-full

# Con rosdep install --from-path src -yi instalamos las depencias necesarias declaradas
# en los paquetes del directorio src, si quieremos construir la imagen Docker, 
# como no tenemos el workspace con el repositorio ejecutamos rosdep install --from-path src -i -y --simulate
# para conocer qué dependencias hay que instalar y añadirlas aquí

# Instalar dependencias adicionales
RUN apt-get update && apt-get install -y \
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
    ros-humble-nav2-bringup

RUN git clone https://github.com/osrf/gazebo_models.git /usr/share/gazebo/models

# Instalar Python3, pip y pipreqs
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip install pipreqs

# Instalar dependencias Python repo create3
RUN pip install --no-cache-dir \
    setuptools==59.6.0 \
    pipreqs

# Copiar scripts o configuraciones
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Variables de entorno
# ENV ROS_WS=/opt/ros_ws
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo/models

ENTRYPOINT ["/ros_entrypoint.sh"]

# Comando por defecto
CMD ["/bin/bash"]

# docker build -t ucm_tfm:humble .
