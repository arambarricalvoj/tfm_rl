# MAP_PROCESSOR 
Es un paquete que he construido para programar y depurar cosas relacionadas con los mapas y SLAM. Actualmente tiene dos funciones principales: 
- ``map_proc_ws/src/map_processor/map_processor/map_node.py`` que se compila como ``reset_slam``, es el nodo que se encarga de ejecutar SLAM y reiniciarlo cuando se reinician los episodios.

**Importante**: el control c al matar luego control z y ps aux y kill -9 PID PID2 PID3...

- el fichero ``map_proc_ws/src/map_processor/map_processor/map_processing_utils.py`` que es donde está programada la lógica de los mapas con sus respectivos *downsamplin* y preprocesados. Es código hecho con IA porque aunque no es complicado, sí es muy tedioso. Por eso, cuando se ejecuta en la fase de depuración se visualiza una ventana gráfica con los mapas para comprobar que sean tal como los queremos. En el fichero ``README_borrado.md`` están los pasos para ejecutarlo.

**Importante**: el fichero ``map_processing_utils.py`` se ha actualizado para quitar el mapa probabilístico de incertidumbre y el local para poner uno de ocupación. Por eso hay muchas partes comentadas. He puesto variables globales con las que se pueden elegir qué mapas se quieren, se definen en el ``__init__(self)``:
```python
# Flags para activar/desactivar mapas derivados
        self.enable_prob_map = True # Proba. Incertidumbre
        self.enable_lem = True # Local egocentric map
        self.enable_global_reduced_map = True # Ocupación actual
```

Lo ideal sería gestionarlo por parámetros, pero está construido sobre la marcha. 

El otro aspecto importante de este fichero es que una vez funcione como queremos, hay que copiarlo a ``/home/$USER/turtlebot3_drlnav_ws/src/turtlebot3_drl/drl_environment`` para que el fichero ``drl_environment.py`` pueda utilizarlo.

Todo esto ya está hecho de manera que con ejecutar lo del README principal ya funciona.

<br>

# CREATE3_WS
Modificaciones en .model de Gazebo para que funcione en Docker. Ahora los mundos a cargar son:
- Sin obstáculos dinámicos: ``/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9.model``
- Con obstáculos dinámicos: ``/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9-obstacle-javi.model``

Nueva configuración de la toolbox de SLAM:
- ``create3_ws/src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml``

<br>

# TURTLEBOT3_DRLNAV
He modificado los mismo ficheros que me vosotros, cuando correspondía.
Además, hay ficheros "nuevos" que son copias de seguridad, suelen llevar el sufijo ``_original`` o ``_raul``. 

En el caso de ``td3.py`` existe también el ``_probMap.py``, que corresponden con la versión del mapa de incertidumbre y local de ocupación que propusimos al principio. ¡OJOª Luego me di cuenta de que la CNN del mapa no era la mejor, pero peor aún, que la del crítico era diferente a la del actor, con razón era inestable... Ya está corregido en el ``td3.py``.

Por otro lado, es posible que en los códigos haya comentarios, variables que no se usen... "basura". Esto se debe a que he hecho infinidad de pruebas y no está limpio. Por ejemplo, en ``drl_gazebo`` (gazebo_goals) creo que sigo generando los goals, porque como la idea final era juntar exploración con navegación... Sin embargo, aunque todo eso sobre para la parte de exploración, como no afecta, lo he dejado para intentar mantener el código lo más parecido posible al original.