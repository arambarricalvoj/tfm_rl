# README Javi

Se ha creado una imagen Docker para encapsular la aplicación. 
La imagen se construye situando la terminar en este directorio y ejecutando:

```bash
docker build -t ucm_tfm:humble .
```
# Pasos para ejecutar

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

--



La imagen construida ya incorpora todas las dependencias (ver el Dockerfile)

Después, ejecutar ``./run.sh`` para arrancar el contenedor con el repositorio ya montado, y dentro del contenedor en ``/home/$USER/create3_ws/``

He seguido la guía de Raúl para la parte de SLAM y OK para el teleop. Interesante añadir un joy para un mando?
La parte de NAV2 da error: 
[planner_server-3] [INFO] [1764257758.514486365] [global_costmap.global_costmap]: Timed out waiting for transform from base_link to map to become available, tf error: Could not find a connection between 'map' and 'base_link' because they are not part of the same tree.Tf has two or more unconnected trees.

[planner_server-3] [ERROR] [1764257761.651344390] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
[planner_server-3] [ERROR] [1764257766.651217621] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
[planner_server-3] [WARN] [1764257769.014886227] [global_costmap.global_costmap]: Can't update static costmap layer, no map received
[planner_server-3] [ERROR] [1764257771.651237590] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
[planner_server-3] [ERROR] [1764257776.651144170] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
[planner_server-3] [WARN] [1764257779.014949861] [global_costmap.global_costmap]: Can't update static costmap layer, no map received
[planner_server-3] [ERROR] [1764257781.651110232] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
[planner_server-3] [ERROR] [1764257786.651094432] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
[planner_server-3] [WARN] [1764257789.014938636] [global_costmap.global_costmap]: Can't update static costmap layer, no map received

He llegado hasta arrancar con Gazebo Classic (funciona).
Me he quedado con el error al poner dos robots.
Y AWS House no he hecho.

Con Ignition da un error de "False" is not in choiches "['true', 'false']" y al modificar los launchers seguía adelante, muy despacio y con muchos warnings (y seguramente errores) y se me ha bloqueado el ordenador. Igual es porque no he seguido estos pasos: 
Ignition Fortress
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress

--

[gzserver-1] [ERROR] [1764089779.366917344] [gazebo_ros_node]: Found multiple nodes with same name: /laser_controller. This might be due to multiple plugins using the same name. Try changing one of the the plugin names or use a different ROS namespace. This error might also result from a custom plugin inheriting from another gazebo_ros plugin and the custom plugin trying to access the ROS node object hence creating multiple nodes with same name. To solve this try providing the optional node_name argument in gazebo_ros::Node::Get() function. [ERROR] [gzserver-1]: process has died [pid 748, exit code -11, cmd 'gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so extra-gazebo-args --ros-args --params-file /home/javierac/create3_ws/install/irobot_create_gazebo_bringup/share/irobot_create_gazebo_bringup/config/gazebo_params.yaml'].
Ese error de Gazebo/ROS2 es bastante típico cuando se cargan dos plugins que intentan crear un nodo ROS con el mismo nombre (/laser_controller en tu caso).

🔹 Qué significa el mensaje
Gazebo está lanzando varios plugins (por ejemplo, controladores de sensores, controladores de joints, etc.).

Dos de ellos llaman a gazebo_ros::Node::Get() sin especificar un node_name, lo que hace que ambos intenten registrarse como /laser_controller.

ROS2 no permite que existan dos nodos con el mismo nombre, así que se produce el conflicto y Gazebo termina con un segfault (exit code -11).

🔹 Causas habituales
Múltiples plugins en el mismo modelo que usan el mismo ros__parameters o node_name.

URDF/SDF duplicado: tu robot puede tener dos <gazebo_ros> plugins configurados con el mismo nombre.

Bringup duplicado: el paquete irobot_create_gazebo_bringup puede estar lanzando un plugin y tu propio launch otro, ambos con /laser_controller.

Custom plugin heredado: si has extendido un plugin de gazebo_ros, puede estar intentando acceder al mismo nodo base.

🔹 Cómo solucionarlo
Revisa tu archivo SDF/URDF: busca las secciones <plugin> que usan gazebo_ros y asegúrate de que cada una tenga un node_name distinto. Ejemplo:

xml
<plugin name="laser_controller" filename="libgazebo_ros_laser.so">
  <ros>
    <namespace>/robot1</namespace>
    <argument>node_name:=laser_controller_1</argument>
  </ros>
</plugin>
Usa namespaces diferentes: si tienes varios robots o sensores, ponlos en /robot1/laser_controller y /robot2/laser_controller.

Configura gazebo_params.yaml: en ese archivo puedes dar nombres distintos a cada plugin.

Si es un plugin custom: en el código, cuando llames a gazebo_ros::Node::Get(), pasa el argumento node_name para diferenciarlo.

🔹 Pasos prácticos
Abre /home/javierac/create3_ws/install/irobot_create_gazebo_bringup/share/irobot_create_gazebo_bringup/config/gazebo_params.yaml.

Busca la sección que define laser_controller.

Cambia el node_name o añade un namespace.

Reconstruye el workspace (colcon build) y vuelve a lanzar Gazebo.

✅ En resumen: El error ocurre porque dos plugins intentan usar el mismo nombre de nodo /laser_controller. La solución es darles nombres únicos o namespaces distintos en el URDF/SDF o en el archivo de parámetros.

pero estoy usando namespace: ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py namespace:=robot1; y ros2 launch irobot_create_gazebo_bringup create3_spawn.launch.py namespace:=robot2 x:=1.0
Vale, aquí el detalle importante: aunque estés lanzando cada robot con un namespace distinto (robot1, robot2), el error que ves (Found multiple nodes with same name: /laser_controller) indica que los plugins de Gazebo están creando nodos con nombre fijo y no están respetando el namespace que les pasas desde el launch.

🔹 Por qué ocurre
Los plugins de gazebo_ros (por ejemplo el del láser) crean un nodo ROS interno.

Si el plugin no recibe explícitamente el parámetro node_name o no está configurado para usar el namespace, todos los robots terminan con un nodo llamado /laser_controller.

Al coexistir dos instancias en el mismo proceso de gzserver, se produce el conflicto.

🔹 Cómo solucionarlo
Revisar el SDF/URDF del robot

Busca la sección del plugin del láser, algo como:

xml
<plugin name="laser_controller" filename="libgazebo_ros_laser.so">
  <ros>
    <namespace>robot1</namespace>
  </ros>
</plugin>
Asegúrate de que cada robot tenga un namespace distinto y que el plugin use un node_name único:

xml
<plugin name="laser_controller" filename="libgazebo_ros_laser.so">
  <ros>
    <namespace>robot1</namespace>
    <argument>node_name:=laser_controller_robot1</argument>
  </ros>
</plugin>
Configurar gazebo_params.yaml

En el paquete irobot_create_gazebo_bringup hay un gazebo_params.yaml.

Ahí puedes añadir node_name para cada plugin, por ejemplo:

yaml
laser_controller:
  ros__parameters:
    node_name: laser_controller_robot1
Usar static_transform_publisher si es necesario

Si el problema es solo de nombres de frames, puedes añadir un static_transform_publisher para diferenciar los frames de cada robot.

Pero en tu caso el error es de nodos duplicados, así que lo crítico es cambiar node_name.

🔹 En resumen
Aunque uses namespace:=robot1 y namespace:=robot2, los plugins de Gazebo están creando nodos con el mismo nombre fijo (/laser_controller). La solución es darles un node_name único en el SDF/URDF o en el archivo de parámetros para que cada instancia tenga su propio nodo dentro de su namespace.

javierac@localhost:~/Documents/ucm/TFM-PFM/create3_sim_JavAram/create3_ws$ find . -name gazebo_params.yaml
./install/irobot_create_gazebo_bringup/share/irobot_create_gazebo_bringup/config/gazebo_params.yaml
./src/irobot_create_gazebo/irobot_create_gazebo_bringup/config/gazebo_params.yaml