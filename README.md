Esta rama integra la toolbox de *SLAM* en el entrenamiento *DRL*. Para ello, es necesario utilizar la versión/rama *`humble-lifecycle`* de la toolbox de *SLAM* con el objetivo de reiniciar el mapa entre episodios del entrenamiento. Esta versión se encuentra en [https://github.com/SteveMacenski/slam_toolbox/tree/humble_lifecycle](https://github.com/SteveMacenski/slam_toolbox/tree/humble_lifecycle).

El entrenamiento actual del sistema *DRL* está en desarrollo y todavía no utiliza esa versión. Por el momento, solo se exponen los pasos para ejecutar el repositorio del ``create3`` de las instrucciones de Raúl (fichero ``doc/Guía de uso para la simulación Create3.pdf``) con la toolbox de SLAM en su versión *`humble-lifecycle`*.
<br><br>

# 1. Construir la imagen de Docker o descargarla 
Construir:
```bash
docker build -t tfm_rl:slam_toolbox_lifecycle .
```

Descargar:
```bash
docker pull arambarricalvoj/tfm_rl:slam_toolbox_lifecycle
```

Esta imagen Docker incluye todas las dependencias necesarias para ejecutar ambos repositorios (``create3`` y ``drl``) y además sustituye la toolbox de *SLAM* clásica por su versión *`humble-lifecycle`*. 

Esta versión no puede instalarse vía *apt*, por lo que tiene que ser compilada desde el código fuente. Se ha decidido instalarla en ``/ros2_ws`` y cuando se ejecuta ``docker run`` se carga automáticamente. En el resto de terminales accedidas mediante ``docker exec`` es necesario ejecutar ``source /opt/ros/humble/setup.bash`` y ``source /ros2_ws/install/setup.bash``.
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

# 3. SLAM (create3):
En primer lugar, recordar cargar los paquetes de *ROS2* y *slam_toolbox* en su versión *lifecycle*:
```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

Si los paquetes del ``create3`` no están compilados, compilar:
```bash
cd /home/$USER/create3_ws/
colcon build --symlink-install 
```

Cargar los paquetes compilados:
```bash
source /home/$USER/create3_ws/install/setup.bash
```

# MAP PROCESSOR (map_processor):
```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /home/$USER/create3_ws/install/setup.bash
source turtlebot3_drlnav_ws/setup_drlnav.sh
source turtlebot3_drlnav_ws/install/setup.bash
source /home/javierac/map_proc_ws/install/setup.bash

ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9.model

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=create3_ws/src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run map_processor map_processor
```

## Ejecutar la aplicación
Recordar cargar todos los paquetes en cada una de las terminales que se va a usar y posicionarse en ``/home/$USER/create3_ws/``:
```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /home/$USER/create3_ws/install/setup.bash
cd /home/$USER/create3_ws/
```

Primera terminal:
```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
```

Segunda terminal:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml
```
La gestión del ciclo de vida (*lifecycle*) de la toolbox de *SLAM* se explica en el Anexo I, junto con la instalación manual de la versión correspondiente de la toolbox.

Tercera terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Cuarta terminal:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

Quinta terminal:
```bash
ros2 run gazebo_ros spawn_entity.py -entity caja1 -database cardboard_box -x 3 -y 3 -z 0.0
```

Tras haber movido el robot con con el teclado y haber generado un mapa, se pueden mandar *2D Goal pose* desde *RViz2* y *Nav2* (cuarta terminal) se encargará de navegar hasta el objetivo.

# 4. DRL (turtlebot3)
Falta de documentar bien esta sección, solo he añadido la quinta terminal, que estando en ``create3_ws/`` hay que ejecutar SLAM:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml
```
Salen errores de TF timestamp... pero para computación no los necesitamos, por lo que los ignoramos y quitamos la visualización en RViz (a ver si así salen menos). También hay que poner un wait al robot entre episodios para que le dé tiempo a llegar el primer mapa y reconfigurar SLAM para que aunque no sea tan preciso mande el mapa cada menos tiempo y tener más información instantánea.

Lo he solucionado corrigiendo el nombre del parámetro que hay que pasar a la toolbox: antes estaba puesto ``params_file:=`` que no existía y por tanto, cargaba la configuración por defecto, el nombre del parámetro es: ``slam_params_file:=``. 

Además, en el fichero de parámetros he puesto ``transform_publish_period: 0.0`` y ya no publica TF y por tanto no salen esos errores. No se ve la visualización en RViz, pero no hace falta, además que lo único que interesa es el mapa, no el TF. Tampoco se podrá hacer navegación ni localización global, pero creo que lo único que necesitamos es la señal del mapa para el DRL.

Y recordar hacer todos los source:
```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /home/$USER/create3_ws/install/setup.bash
source turtlebot3_drlnav_ws/setup_drlnav.sh
source turtlebot3_drlnav_ws/install/setup.bash
```



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
<br>

# Anexo I: toolbox *SLAM*, rama `humble_lifecycle` — ROS 2 Humble  
## Instalación
### 1. Clonar la rama correcta

```bash
cd ~/ros2_ws/src
git clone -b humble_lifecycle https://github.com/SteveMacenski/slam_toolbox.git
```

### 2. Instalar dependencias
```bash
cd slam_toolbox/
rosdep install -q -y -r --from-paths src --ignore-src
```
### 3. Compilar
```bash
cd slam_toolbox/
colcon build --symlink-install
```
Activar el workspace:
```bash
source install/setup.bash
```

<!-- 
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
-->
<br>

## Gestión del ciclo de vida
Cuando se ejecuta (segunda terminal):
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml
```

El nodo de la toolbox se configura y activate automáticamente, por lo que está funcionando como en la versión sin *lifecycle*.

En mi TFG usé y utilicé los nodos *lifecycle*, por lo que lo expliqué con detalle en mi memoria. Está disponible en el siguiente link: [https://github.com/arambarricalvoj/percepcion-control-ros2-tfg](https://github.com/arambarricalvoj/percepcion-control-ros2-tfg). 

Concretamente, la explicación se encuentra en las páginas 68 y 69, y un ejemplo de cómo hacer que un nodo transicione desde código Python en las líneas 69 a 74 (función ``nodoa_kudeatu``, en castellano: gestionar nodo), cuya definición está en las líneas 81 a 98 de la página 191. La definición de un nodo *lifecycle*, con los métodos específicos ``on_configure, on_activate, on_deactivate, on_cleanup, on_shutdown`` se puede ver en el código de las páginas 181 y 182 de la memoria. Concretamente, es el nodo de la cámara, que actuaba de la siguiente manera (debido a las limitaciones de hardware que tuve, tenía que funcionar así): cuando estaba configurado, se abría la cámara y podías llamar a un servicio para sacar una foto; cuando estaba activado, retrasmitía la imagen en tiempo real (vídeo) pero no podías pedir una foto concreta; cuando se desactivaba, se cancelaba el timer del vídeo y se volvía a activar el servicio para sacar foto; cuando cleanup, se libera la cámara.

Cuando un nodo *lifecycle* está en ejecución, en este caso, la segunda terminal, para ver las transiciones disponibles:
```bash
ros2 lifecycle list /slam_toolbox
```

Para listar los nodos *lifecycle* disponibles: 
```bash
ros2 lifecycle nodes
```

Para desactivar el nodo:
```bash
ros2 lifecycle set /slam_toolbox deactivate
```
Ahora el nodo está "vivo", pero no hace nada, está en pausa. Si ahora ejecutamos ``ros2 lifecycle set /slam_toolbox activate``, el nodo vuelve a funcionar en el punto en el que se desactivó, mantiene toda la información que tenía (mapa).

Para "limpiar/borrar" la información del nodo (el mapa), con el objetivo de reiniciarlo:
```bash
ros2 lifecycle set /slam_toolbox cleanup
```

Para volver a configurar el nodo:
```bash
ros2 lifecycle set /slam_toolbox configure
```
**La toolbox de *SLAM* *lifecycle* se activa automáticamente cuando se configura, por lo que no es necesario ejecutar ``ros2 lifecycle set /slam_toolbox configure`` después del ``configure``.**

Cuando hemos limpiado el nodo (después de ``cleanup``), si queremos apagarlo/matarlo:
```bash
ros2 lifecycle set /slam_toolbox shutdown
```
**OJO: esto hace que el nodo pase a su estado final, que no acepta más transiciones, y por tanto, ya no se puede reiniciar, habría que matar el proceso por terminal y volver a arrancar el *launch***.

<!-- Para volver a usarlo, relanza el launch.

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
```-->

## Resumen del ciclo de vida
```bash
ros2 lifecycle get /slam_toolbox # Ver estado actual
ros2 lifecycle list /slam_toolbox # Ver transiciones posibles actuales
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle set /slam_toolbox deactivate
ros2 lifecycle set /slam_toolbox cleanup
ros2 lifecycle set /slam_toolbox shutdown
```