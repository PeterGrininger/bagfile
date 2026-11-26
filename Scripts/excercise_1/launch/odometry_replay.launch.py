"""
Replay a driving bag, compute own odometry, log trajectories, show RViz.

  ros2 launch excercise_1 odometry_replay.launch.py bag:=<bag folder>

The recorded TF odom->base_footprint is broken in our bags, so /tf is replayed
as /tf_bag, filtered by tf_filter, and odom->base_footprint is published by
our own my_odometry node instead. In RViz (fixed frame "odom") the robot then
moves according to OUR odometry and the laser scans show how well it fits.

Launch arguments (name:=value)
  bag          bag folder
  output_file  csv for the trajectory_logger (default: ./Results/trajectories.csv,
               run from the repo root)
  method       approx | exact (odometry integration)
  rviz, rate   see laser_replay.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    bag = LaunchConfiguration('bag').perform(context)
    if not bag:
        raise RuntimeError('Please pass bag:=<path to bag folder>')
    out = os.path.abspath(os.path.expanduser(LaunchConfiguration('output_file').perform(context)))
    sim = {'use_sim_time': True}
    return [
        Node(package='excercise_1', executable='my_odometry', output='screen',
             parameters=[sim, {'publish_tf': True,
                               'method': LaunchConfiguration('method').perform(context)}]),
        Node(package='excercise_1', executable='trajectory_logger', output='screen',
             parameters=[sim, {'output_file': out}]),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-i', os.path.abspath(os.path.expanduser(bag)),
                 '--clock', '--delay', '3',
                 '--rate', LaunchConfiguration('rate').perform(context),
                 '--remap', '/tf:=/tf_bag'],
            output='screen', name='bag_play'),
    ]


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('excercise_1'), 'rviz', 'odometry.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('output_file', default_value='Results/trajectories.csv'),
        DeclareLaunchArgument('method', default_value='approx'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rate', default_value='1.0'),

        Node(package='excercise_1', executable='tf_filter', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        OpaqueFunction(function=_setup),
    ])
