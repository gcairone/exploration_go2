export TURTLEBOT3_MODEL=burger
Turtlebot
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
	ros2 launch turtlebot3_bringup rviz2.launch.py
	ros2 run turtlebot3_teleop teleop_keyboard
	
Nav2
   	apt update && apt install ros-foxy-turtlebot3-navigation2 -y
	ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
	
	



root@pc-huawei:~# python3 /home/share/topic_info.py 
📋 Recupero lista dei topic ROS 2...


🔍 Informazioni per il topic: /amcl/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /amcl_pose
Type: geometry_msgs/msg/PoseWithCovarianceStamped
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /bt_navigator/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /clicked_point
Type: geometry_msgs/msg/PointStamped
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /clock
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
Subscription count: 29

🔍 Informazioni per il topic: /cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 4
Subscription count: 1

🔍 Informazioni per il topic: /controller_server/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /cost_cloud
Type: sensor_msgs/msg/PointCloud
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /downsampled_costmap
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /downsampled_costmap_updates
Type: map_msgs/msg/OccupancyGridUpdate
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /evaluation
Type: dwb_msgs/msg/LocalPlanEvaluation
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /global_costmap/costmap
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /global_costmap/costmap_raw
Type: nav2_msgs/msg/Costmap
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /global_costmap/costmap_updates
Type: map_msgs/msg/OccupancyGridUpdate
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /global_costmap/footprint
Type: geometry_msgs/msg/Polygon
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /global_costmap/global_costmap/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /global_costmap/published_footprint
Type: geometry_msgs/msg/PolygonStamped
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /global_costmap/voxel_marked_cloud
Type: sensor_msgs/msg/PointCloud
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /goal_pose
Type: geometry_msgs/msg/PoseStamped
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /imu
Type: sensor_msgs/msg/Imu
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /initialpose
Type: geometry_msgs/msg/PoseWithCovarianceStamped
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /joint_states
Type: sensor_msgs/msg/JointState
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /local_costmap/clearing_endpoints
Type: sensor_msgs/msg/PointCloud
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /local_costmap/costmap
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /local_costmap/costmap_raw
Type: nav2_msgs/msg/Costmap
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /local_costmap/costmap_updates
Type: map_msgs/msg/OccupancyGridUpdate
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /local_costmap/footprint
Type: geometry_msgs/msg/Polygon
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /local_costmap/local_costmap/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /local_costmap/published_footprint
Type: geometry_msgs/msg/PolygonStamped
Publisher count: 1
Subscription count: 2

🔍 Informazioni per il topic: /local_costmap/voxel_grid
Type: nav2_msgs/msg/VoxelGrid
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /local_costmap/voxel_marked_cloud
Type: sensor_msgs/msg/PointCloud
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /local_plan
Type: nav_msgs/msg/Path
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /map
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Subscription count: 3

🔍 Informazioni per il topic: /map_server/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /map_updates
Type: map_msgs/msg/OccupancyGridUpdate
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /marker
Type: visualization_msgs/msg/MarkerArray
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /mobile_base/sensors/bumper_pointcloud
Type: sensor_msgs/msg/PointCloud2
Publisher count: 0
Subscription count: 1

🔍 Informazioni per il topic: /odom
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /parameter_events
Type: rcl_interfaces/msg/ParameterEvent
Publisher count: 34
Subscription count: 40

🔍 Informazioni per il topic: /particle_cloud
Type: nav2_msgs/msg/ParticleCloud
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /particlecloud
Type: geometry_msgs/msg/PoseArray
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /performance_metrics
Type: gazebo_msgs/msg/PerformanceMetrics
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /plan
Type: nav_msgs/msg/Path
Publisher count: 1
Subscription count: 1

🔍 Informazioni per il topic: /planner_server/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /received_global_plan
Type: nav_msgs/msg/Path
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /recoveries_server/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /robot_description
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /rosout
Type: rcl_interfaces/msg/Log
Publisher count: 36
Subscription count: 0

🔍 Informazioni per il topic: /scan
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 4

🔍 Informazioni per il topic: /tf
Type: tf2_msgs/msg/TFMessage
Publisher count: 3
Subscription count: 6

🔍 Informazioni per il topic: /tf_static
Type: tf2_msgs/msg/TFMessage
Publisher count: 1
Subscription count: 6

🔍 Informazioni per il topic: /transformed_global_plan
Type: nav_msgs/msg/Path
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /waypoint_follower/transition_event
Type: lifecycle_msgs/msg/TransitionEvent
Publisher count: 1
Subscription count: 0

🔍 Informazioni per il topic: /waypoints
Type: visualization_msgs/msg/MarkerArray
Publisher count: 1
Subscription count: 1

