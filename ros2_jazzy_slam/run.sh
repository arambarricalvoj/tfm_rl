xhost +local:*
docker run -e DISPLAY=$DISPLAY \
           -e USER=$USER \
           -e QT_X11_NO_MITSHM=1 \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -v ./mapper_params_online_async.yaml:/home/$USER/mapper_params_online_async.yaml \
           -it \
           --rm \
           --network host \
           --gpus all \
           --runtime=nvidia \
           --name slam_ros2_jazzy_tfm_rl \
           tfm_rl:ros2_jazzy_slam
           #tfm_rl:slam_toolbox_lifecycle
           # tfm_rl:latest
           # -v ./slam_toolbox:/home/$USER/slam_toolbox/ \
