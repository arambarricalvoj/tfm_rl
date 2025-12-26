xhost +local:*
docker run -e DISPLAY=$DISPLAY \
           -e USER=$USER \
           -e QT_X11_NO_MITSHM=1 \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -v ./create3_ws/:/home/$USER/create3_ws/ \
           -it \
           --rm \
           --gpus all \
           --runtime=nvidia \
           --name tfm \
           ucm_tfm:humble

# --device /dev/dri:/dev/dri \
