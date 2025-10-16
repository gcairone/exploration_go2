#!/bin/bash
# Script per copiare file di configurazione ROS2

set -e  # Termina lo script in caso di errore

# Copia bringup_launch.py e nav2_params.yaml
if [ -f /home/share/bringup_launch.py ]; then
    mkdir -p /opt/ros/foxy/share/nav2_bringup/launch
    cp /home/share/bringup_launch.py /opt/ros/foxy/share/nav2_bringup/launch/bringup_launch.py
    echo "Copied bringup_launch.py"
else
    echo "File bringup_launch.py non trovato!"
fi

if [ -f /home/share/nav2_params.yaml ]; then
    mkdir -p /opt/ros/foxy/share/nav2_bringup/params
    cp /home/share/nav2_params.yaml /opt/ros/foxy/share/nav2_bringup/params/nav2_params.yaml
    echo "Copied nav2_params.yaml"
else
    echo "File nav2_params.yaml non trovato!"
fi

# Copia online_async_launch.py e mapper_params_online_async.yaml
if [ -f /home/share/online_async_launch.py ]; then
    mkdir -p /opt/ros/foxy/share/slam_toolbox/launch
    cp /home/share/online_async_launch.py /opt/ros/foxy/share/slam_toolbox/launch/online_async_launch.py
    echo "Copied online_async_launch.py"
else
    echo "File online_async_launch.py non trovato!"
fi

if [ -f /home/share/mapper_params_online_async.yaml ]; then
    mkdir -p /opt/ros/foxy/share/slam_toolbox/config
    cp /home/share/mapper_params_online_async.yaml /opt/ros/foxy/share/slam_toolbox/config/mapper_params_online_async.yaml
    echo "Copied mapper_params_online_async.yaml"
else
    echo "File mapper_params_online_async.yaml non trovato!"
fi

# Copia /home/share/burger.model in /opt/ros/foxy/share/turtlebot3_gazebo/worlds/turtlebot3_worlds/burger.model se esiste
if [ -f /home/share/burger.model ]; then
    mkdir -p /opt/ros/foxy/share/turtlebot3_gazebo/worlds/turtlebot3_worlds
    cp /home/share/burger.model /opt/ros/foxy/share/turtlebot3_gazebo/worlds/turtlebot3_worlds/burger.model
    echo "Copied burger.model"
else
    echo "File burger.model non trovato!"
fi

# installa se non è installato python3-colcon-common-extensions -y
apt update
if ! dpkg -s python3-colcon-common-extensions >/dev/null 2>&1; then
    apt install python3-colcon-common-extensions -y
    echo "Installed colcon"
else
    echo "python3-colcon-common-extensions già installato"
fi

# installa se non è installato build-essential -y
if ! dpkg -s build-essential >/dev/null 2>&1; then
    apt install build-essential -y
    echo "Installed build-essential"
else
    echo "build-essential già installato"
fi

# installa se non è installato pcl-ros pcl-conversions -y
if ! dpkg -s ros-foxy-pcl-ros >/dev/null 2>&1; then
    apt install ros-foxy-pcl-ros -y
    echo "Installed pcl-ros"
else
    echo "pcl-ros già installato"
fi
if ! dpkg -s ros-foxy-pcl-conversions >/dev/null 2>&1; then
    apt install ros-foxy-pcl-conversions -y
    echo "Installed pcl-conversions"
else
    echo "pcl-conversions già installato"
fi


#apt update && apt install python3-colcon-common-extensions -y
#echo "Installed colcon"
