Aunque falta documentar mejor y ordenar el repo, ejecutando estos pasos funciona :) !

## Construir la imagen de Docker o descargarla 
Construir:
```bash
docker build -t tfm_rl:latest
```

Descargar:
```bash
docker pull arambarricalvoj/tfm_rl:latest
```

## Ejecutar contenedor de Docker
```bash
sudo chmod u+x run.sh
./run.sh
```

Para acceder al contenedor desde otras terminales:
```bash
docker exec -it tfm_rl bash
```

## SLAM (create3):

```bash
source /home/$USER/create3_ws/install/setup.bash
```

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
```

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True params_file:=src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

```bash
ros2 run gazebo_ros spawn_entity.py -entity caja1 -database cardboard_box -x 3 -y 3 -z 0.5
```

Ejecutar en RViz2 con 2D Goal pose... 

## DRL-NAV (turtlebot3)
```bash
source /home/$USER/turtlebot3_drlnav_ws/install/setup.bash
source /home/$USER/turtlebot3_drlnav_ws/setup_drlnav.sh 
```

```bash
touch /tmp/drlnav_current_stage.txt
echo 9 > /tmp/drlnav_current_stage.txt
```

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9.model
```
o
```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9-obstacle-javi.model
```

```bash
ros2 run turtlebot3_drl environment
```

```bash
ros2 run turtlebot3_drl train_agent ddpg
```

```bash
ros2 run turtlebot3_drl gazebo_goals
```

saving data for episode: 1, location: /home/javierac/turtlebot3_drlnav_ws/src/turtlebot3_drl/model/8130905e3c29/ddpg_0_stage_9