#!/bin/bash

# Nome del container
CONTAINER_NAME="turtlebot3_nav2_container"

# Comando da eseguire nel container
COMMAND="export ROS_DOMAIN_ID=0 && export TURTLEBOT3_MODEL=burger && bash"

# Esegui il comando nel container con shell interattiva
docker exec -it "$CONTAINER_NAME" bash -c "$COMMAND"
