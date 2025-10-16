export TURTLEBOT3_MODEL=burger
Turtlebot
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
	export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard
	
Nav2
   	apt update && apt install ros-foxy-turtlebot3-navigation2 -y
	ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

Mio Nav2
	ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
	ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true
	rviz2 -d /home/share/planning.rviz
	apt update && apt install python3-colcon-common-extensions -y

	python3 /home/share/maze_creator/maze_creator.py /home/share/maze_creator/maze.txt /opt/ros/foxy/share/turtlebot3_gazebo/models/turtlebot3_world/model.sdf

Trasf
ros2 run tf2_ros static_transform_publisher 0.3 0 0 0 0.9848 0 0.1736 robot_center utlidar_lidar

Lancio
ros2 run explorer_pkg map_projector --ros-args   --params-file install/explorer_pkg/share/explorer_pkg/config/map_projector_params.yaml
