"""
Replay a (static) bag with RViz and compute the laser statistics.

  ros2 launch excercise_1 laser_replay.launch.py bags:="<bag1> [<bag2> ...]"

Launch arguments (name:=value)
  bags        one or more bag folders (space separated) - played together,
              e.g. bags:="Messaufnahmen/rosbag2_2026_11_12-10_55_22"
  analyze     true: run scan_statistics + scan_sum (writes csv files)
  output_dir  where the csv files are written (default: ./Results, run from the repo root)
  rviz        start RViz (true/false)
  rate        playback rate
  loop        loop playback (do NOT combine with analyze, scans would count twice)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bag_play(context):
    bags = LaunchConfiguration('bags').perform(context).split()
    if not bags:
        raise RuntimeError('Please pass bags:="<path to bag folder> [...]"')
    cmd = ['ros2', 'bag', 'play']
    for b in bags:
        cmd += ['-i', os.path.abspath(os.path.expanduser(b))]
    cmd += ['--clock', '--delay', '3',
            '--rate', LaunchConfiguration('rate').perform(context),
            '--remap', '/tf:=/tf_bag']
    if LaunchConfiguration('loop').perform(context).lower() == 'true':
        cmd.append('--loop')
    return [ExecuteProcess(cmd=cmd, output='screen', name='bag_play')]


def _analysis_nodes(context):
    out = os.path.abspath(os.path.expanduser(LaunchConfiguration('output_dir').perform(context)))
    cond = IfCondition(LaunchConfiguration('analyze'))
    sim = {'use_sim_time': True}
    return [
        Node(package='excercise_1', executable='scan_statistics', output='screen',
             parameters=[sim, {'output_file': os.path.join(out, 'scan_statistics.csv')}],
             condition=cond),
        Node(package='excercise_1', executable='scan_sum', output='screen',
             parameters=[sim, {'output_file': os.path.join(out, 'scan_sums.csv')}],
             condition=cond),
    ]


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('excercise_1'), 'rviz', 'laser.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('bags', default_value=''),
        DeclareLaunchArgument('analyze', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='Results'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),

        Node(package='excercise_1', executable='tf_filter', output='screen',
             parameters=[{'use_sim_time': True}]),
        OpaqueFunction(function=_analysis_nodes),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        OpaqueFunction(function=_bag_play),
    ])
