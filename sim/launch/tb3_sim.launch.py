# TurtleBot3 burger in an arbitrary Gazebo world, with initial pose and selectable robot model.
#
#   ros2 launch sim/launch/tb3_sim.launch.py world:=<file.world> x_pose:=0 y_pose:=0 yaw:=0 \
#        model:=<model.sdf> gui:=false
#
# model defaults to the stock turtlebot3_burger (LDS, 3.5 m range); sim/models/turtlebot3_burger_lr
# is the same robot with a 12 m lidar. gui:=true also starts gzclient (needs a display).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz = get_package_share_directory('gazebo_ros')
    default_model = os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf')
    default_world = os.path.join(tb3, 'worlds', 'turtlebot3_world.world')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'burger', '-file', LaunchConfiguration('model'),
                        '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
                        '-z', '0.01', '-Y', LaunchConfiguration('yaw')]),
    ])
