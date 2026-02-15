#!/bin/bash
xhost +local:docker > /dev/null

IMAGE_NAME="nav2_jazzy"
CONTAINER_NAME="nav2_dev"

if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
  echo "Image not found. Building $IMAGE_NAME..."
  docker build -t $IMAGE_NAME .
fi

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container is already running. Entering..."
    docker exec -it $CONTAINER_NAME bash
elif [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Starting existing container..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    echo "Launching fresh container with GPU & Signal support..."
    docker run -it \
        --name $CONTAINER_NAME \
        --init \
        --network host \
        --privileged \
        --gpus all \
        --env="DISPLAY=$DISPLAY" \
        --env="NVIDIA_VISIBLE_DEVICES=all" \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$(pwd):/ros2_ws/src/Mobile-Robot-Collision" \
        --volume="$(pwd)/build:/ros2_ws/build" \
        --volume="$(pwd)/install:/ros2_ws/install" \
        --volume="$(pwd)/log:/ros2_ws/log" \
        $IMAGE_NAME
fi
