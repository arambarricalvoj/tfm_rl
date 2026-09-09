xhost +local:*
docker run -e DISPLAY=$DISPLAY \
           -e USER=$USER \
           -e QT_X11_NO_MITSHM=1 \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -v ./create3_sim_JavAram/create3_ws/:/home/$USER/create3_ws/ \
           -v ./turtlebot3_drlnav_JavAram:/home/$USER/turtlebot3_drlnav_ws/ \
           -v ./map_proc_ws:/home/$USER/map_proc_ws/ \
           -v ./ros2_ws_rpy:/home/$USER/rpy_utils\
           -it \
           --rm \
           --network host \
           --gpus all \
           --runtime=nvidia \
           --name tfm_rl \
           arambarricalvoj/tfm_rl:slam_apt
           #tfm_rl:slam_toolbox_lifecycle
           # tfm_rl:latest
           # -v ./slam_toolbox:/home/$USER/slam_toolbox/ \
