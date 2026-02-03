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

## Descargar explore_lite
```bash
cd create3_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
colcon build --symlink-install
source ...
ros2 launch m_explore explore.launch.py

```

```bash
# guardar maoa
ros2 run nav2_map_server map_saver_cli -f my_map
```