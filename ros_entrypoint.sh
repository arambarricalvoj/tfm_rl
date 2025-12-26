#!/bin/bash
set -e

# Cargar entorno de ROS
source /opt/ros/humble/setup.bash

# Si tienes un workspace, también:
# source /opt/ros_ws/install/setup.bash
#source /home/$USER/create3_ws/install/setup.bash
#source /home/$USER/turtlebot3_drlnav_ws/install/setup.bash
#source /home/$USER/turtlebot3_drlnav_ws/setup_drlnav.sh 

# Ejecutar el comando que se pase al contenedor
exec "$@"
