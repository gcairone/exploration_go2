#!/bin/bash


xhost +local:docker


CONTAINER_NAME=turtlebot3_nav2_container


SHARE_PATH=/home/gcairone/TESI/nav2_prove/share
WS_PATH=/home/gcairone/TESI/nav2_prove/ros2_ws

if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  docker run -it -d \
    --name $CONTAINER_NAME \
    --net=host \
    --env=DISPLAY=$DISPLAY \
    --device /dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$SHARE_PATH:/home/share" \
    -v "$WS_PATH:/root/ros2_ws" \
    ros2_foxy:turtlebot3_nav2 \
    /bin/bash
fi



SHELL_CMD="export ROS_DOMAIN_ID=0 && export TURTLEBOT3_MODEL=burger && exec bash"

gnome-terminal -- bash -c "docker exec -it $CONTAINER_NAME bash -c '$SHELL_CMD'"
gnome-terminal -- bash -c "docker exec -it $CONTAINER_NAME bash -c '$SHELL_CMD'"


