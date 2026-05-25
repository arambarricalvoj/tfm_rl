# 1. Construir la imagen de Docker o descargarla 
Construir:
```bash
docker build -t tfm_rl:slam_apt .
```

Descargar:
```bash
docker pull arambarricalvoj/tfm_rl:slam_apt
```

Esta imagen Docker incluye todas las dependencias necesarias para ejecutar ambos repositorios (``create3`` y ``drl``).
<br><br>

# 2. Ejecutar imagen de Docker
```bash
sudo chmod u+x run.sh
./run.sh
```

Para acceder al contenedor desde otras terminales:
```bash
docker exec -it tfm_rl bash
```

En las terminales accedidas mediante ``docker exec`` es necesario ejecutar ``source /opt/ros/humble/setup.bash`` y ``source /ros2_ws/install/setup.bash``.
<br><br>

# 3. Ejecutar entrenamiento SLAM (Javi)

Dentro de Docker:

Build:
```bash
cd /home/$USER/map_proc_ws/
colcon build

cd /home/$USER/create3_ws/
colcon build

cd /home/$USER/turtlebot3_drlnav_ws/
colcon build
```

Source:
```bash
source /opt/ros/humble/setup.bash
source /home/$USER/create3_ws/install/setup.bash
source /home/$USER/turtlebot3_drlnav_ws/setup_drlnav.sh
source /home/$USER/turtlebot3_drlnav_ws/install/setup.bash #*
source /home/$USER/map_proc_ws/install/setup.bash #*
```

Los comandos marcados con #* probablemente sean prescindibles, pero como funciona así y no he probado de otra manera, lo dejo puesto.

```bash
touch /tmp/drlnav_current_stage.txt
echo 9 > /tmp/drlnav_current_stage.txt
```

## **Todos los comandos de ejecución tienen que ejecutarse desde ``/home/$USER/``, en mi caso ``/home/javierac/``. Si estamos en esa carpeta, al hacer ``ls`` tendrán que salir los 3 workspaces: ``create3_ws/``, ``map_proc_ws``, ``turtlebot3_drlnav_ws``**

Antes de ejecutar, todas las terminales tienen que estar en ``/home/$USER/``:
```bash
cd /home/$USER/
```

Terminal 1:
```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=/home/$USER/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9.model use_gazebo_gui:=false
```

o si queremos el robot en la esquina superior derecha:

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py     world_path:=/home/$USER/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9.model use_gazebo_gui:=false x:=2.25 y:=2.25 z:=0.01 yaw:=-1.57

```

Esto arranca la simulación sin ventana gráfica (solo RViz) y el mundo sin obstáculos dinámicos. Si se quiere arrancar con obstáculos dinámicos cambiar 

``world_path:=/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9.model`` 

por ``world_path:=/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9-obstacle-javi.model``.

Terminal 2:
```bash
ros2 run turtlebot3_drl environment
```

Terminal 3:
```bash
ros2 run turtlebot3_drl train_agent td3
```

Terminal 4:
```bash
ros2 run turtlebot3_drl gazebo_goals
```

Terminal 5:
```bash
ros2 run map_processor reset_slam
```

**IMPORTANTE**: arrancar primero gazebo + rviz y SLAM, y cuando salga el mapa en RVIZ se puede arrancar el resto.

# Inferencia
```bash
ros2 run turtlebot3_drl test_agent td3 td3_13_stage_9 12000
```

# Robot Real

Terminal 1 (Raspberry Pi - no docker) - Run RPLIDAR + tf LIDAR-Robot:
```bash
# Connect to Raspbery Pi (recommended to use ssh)
cd ros2_ws
./launch_slam.sh
```

Terminal 2 (Raspberry Pi - no docker) - Node to transform scan to a fixed 500 samples:
```bash
# Connect to Raspbery Pi (recommended to use ssh)
python3 scan_resampler.py
```

Terminal 3 - Run RL Environment:
```bash
# Run first source commands as indicated above
touch /tmp/drlnav_current_stage.txt
echo 9 > /tmp/drlnav_current_stage.txt
export ROS_DOMAIN_ID=0
ros2 run turtlebot3_drl real_environment
```

Terminal 4 - SLAM:
```bash
# Run first source commands as indicated above
cd /home/$USER/
export ROS_DOMAIN_ID=0
ros2 run slam_toolbox async_slam_toolbox_node   --ros-args   --params-file /home/raul/create3_ws/src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml   -p use_sim_time:=false   -r /tf:=/vin/tf   -r /tf_static:=/vin/tf_static
```

Terminal 5 - Policy:
```bash
# Run first source commands as indicated above
export ROS_DOMAIN_ID=0
ros2 run turtlebot3_drl real_agent td3 'td3_19_stage_9' 5300
```

Terminal X (Optional) - Rviz:
```bash
# Run first source commands as indicated above
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2 -d /home/raul/rviz-config-real.rviz --ros-args -r /tf:=/vin/tf -r /tf_static:=/vin/tf_static
```

Terminal Y (Optional) - Teleoperate movement:
```bash
# Run first source commands as indicated above
export ROS_DOMAIN_ID=0
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/vin/cmd_vel
```

