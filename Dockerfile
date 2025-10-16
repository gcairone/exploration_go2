FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release software-properties-common sudo \
    && locale-gen en_US en_US.UTF-8 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    ros-foxy-gazebo-ros-pkgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Aggiunta dei pacchetti TurtleBot3 e altri strumenti
RUN apt-get update && apt-get install -y \
    ros-foxy-turtlebot3-msgs \
    ros-foxy-turtlebot3-description \
    ros-foxy-turtlebot3-gazebo \
    ros-foxy-turtlebot3-simulations \
    ros-foxy-turtlebot3-bringup \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-turtlebot3-teleop \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    tmux nano \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

WORKDIR /root/ros2_ws
CMD ["/bin/bash"]






