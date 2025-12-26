arrancar slam:

``ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True params_file:=create3_ws/src/irobot_create_common/irobot_create_common_bringup/config/mapper_params_online_async.yaml ``

teleop:

``ros2 run teleop_twist_keyboard teleop_twist_keyboard``

nav2:

``ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True``


el primer comando del RL:

``ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=/home/javierac/create3_ws/src/irobot_create_gazebo/irobot_create_gazebo_bringup/launch/worlds/stage9.model``

Arranca, pero con errores de audio (despreciable)
apt-get update && apt-get install -y alsa-utils

Hay que decidir si con conda o el interprete del sistema, por eso he tenido que modificar cuidadosamente el path en el Dockerfile

segundo comando de RL:

``ros2 run turtlebot3_drl environment``

``export PATH=/opt/conda/envs/turtle3-drlnav/bin:$PATH``

pip install empy==3.3.4
pip install catkin_pkg
pip install lark-parser


export PATH=/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/sbin:/bin


tercer comando:

``ros2 run turtlebot3_drl train_agent ddpg``

cuarto comando:

``ros2 run turtlebot3_drl gazebo_goals``