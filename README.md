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

<br>

# SLAM Toolbox (rama `humble_lifecycle`) — ROS 2 Humble  
Guía de instalación, compilación y uso del ciclo de vida (Lifecycle Nodes)

Este documento explica cómo clonar, compilar y ejecutar `slam_toolbox` en su versión con **Lifecycle Nodes**, así como cómo manipular manualmente las transiciones del ciclo de vida. Ha sido creado con Copilot: primero he estado probando la toolbox con lifecycle y después le he pasado a Copilot los comandos que he ejecutado en orden y me ha generado lo siguiente.

---

## 1. Clonar la rama correcta

```bash
cd ~/ros2_ws/src
git clone -b humble_lifecycle https://github.com/SteveMacenski/slam_toolbox.git
```

## 2. Instalar dependencias
```bash
cd slam_toolbox/
rosdep install -q -y -r --from-paths src --ignore-src
```
## 3. Compilar
```bash
cd slam_toolbox/
colcon build --symlink-install
```
Activar el workspace:
```bash
source install/setup.bash
```

## 4. Ejecutar SLAM Toolbox con Lifecycle
Hay dos formas de lanzar el nodo:

A) Launch que activa automáticamente el nodo
(uso normal)

```bash
ros2 launch slam_toolbox online_async_launch.py use_lifecycle:=true
```
Este launch hace automáticamente: ``configure → activate``
Comprobar estado:
```bash
ros2 lifecycle get /slam_toolbox
```

B) Launch para controlar manualmente el ciclo de vida
(ideal para pruebas)

```bash
ros2 launch slam_toolbox lifecycle_launch.py
```
Estado inicial: ``unconfigured``

## 5. Transiciones del ciclo de vida
Configurar el nodo
```bash
ros2 lifecycle set /slam_toolbox configure
```
Estado esperado: ``inactive``

Activar el nodo
```bash
ros2 lifecycle set /slam_toolbox activate
```
Estado esperado: ``active``

Desactivar el nodo
```bash
ros2 lifecycle set /slam_toolbox deactivate
```

Estado esperado: ```inactive```

Limpiar (borra el mapa y reinicia el nodo)
```bash
ros2 lifecycle set /slam_toolbox cleanup
```

Estado esperado: ```unconfigured```

Apagar el nodo (estado final, no acepta más transiciones)
```bash
ros2 lifecycle set /slam_toolbox shutdown
```
Para volver a usarlo, relanza el launch.

## 6.
<table>
  <thead>
    <tr>
      <th>Transición</th>
      <th>¿Se borra el mapa?</th>
      <th>¿Procesa /scan?</th>
      <th>¿Publica /map?</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>activate</strong></td>
      <td>No</td>
      <td>Sí</td>
      <td>Sí</td>
    </tr>
    <tr>
      <td><strong>deactivate</strong></td>
      <td>No</td>
      <td>No</td>
      <td>No</td>
    </tr>
    <tr>
      <td><strong>cleanup</strong></td>
      <td><strong>Sí</strong></td>
      <td>No</td>
      <td>No</td>
    </tr>
    <tr>
      <td><strong>configure</strong></td>
      <td><strong>Sí</strong></td>
      <td>No</td>
      <td>No</td>
    </tr>
    <tr>
      <td><strong>shutdown</strong></td>
      <td><strong>Sí</strong></td>
      <td>No</td>
      <td>No</td>
    </tr>
  </tbody>
</table>



## 7. Ver transiciones disponibles
```bash
ros2 lifecycle list /slam_toolbox
```

## 8. Resumen rápido de comandos
```bash
# Lanzar con activación automática
ros2 launch slam_toolbox online_async_launch.py use_lifecycle:=true

# Lanzar para controlar manualmente???
ros2 launch slam_toolbox lifecycle_launch.py # no probado
```

## Ciclo de vida
```bash
ros2 lifecycle get /slam_toolbox
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle set /slam_toolbox deactivate
ros2 lifecycle set /slam_toolbox cleanup
ros2 lifecycle set /slam_toolbox shutdown
```