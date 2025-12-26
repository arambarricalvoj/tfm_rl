xhost +local:*
docker run -e DISPLAY=$DISPLAY \
           -e USER=$USER \
           -e QT_X11_NO_MITSHM=1 \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -v ./create3_sim_JavAram/create3_ws/:/home/$USER/create3_ws/ \
           -v ./turtlebot3_drlnav_JavAram:/home/$USER/turtlebot3_drlnav_JavAram_ws/ \
           -it \
           --rm \
           --gpus all \
           --runtime=nvidia \
           --name tfm \
           turtlebot3_drlnav

# --device /dev/dri:/dev/dri \


# --privileged  
