#!/bin/bash
set -e

# Cargar entorno de ROS
source /opt/ros/humble/setup.bash

# Si tienes un workspace, también:
# source /opt/ros_ws/install/setup.bash

# Ejecutar el comando que se pase al contenedor
exec "$@"
