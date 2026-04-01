xhost +local:*
docker run -e DISPLAY=$DISPLAY \
           -e USER=$USER \
           -e QT_X11_NO_MITSHM=1 \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -it \
           --rm \
           --network host \
           --gpus all \
           --runtime=nvidia \
           --name ros2_humble \
           osrf/ros:humble-desktop-full
           #tfm_rl:slam_toolbox_lifecycle
           # tfm_rl:latest
           # -v ./slam_toolbox:/home/$USER/slam_toolbox/ \
