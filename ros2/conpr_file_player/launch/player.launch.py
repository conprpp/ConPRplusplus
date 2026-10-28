"""Installed native Qt/headless player launch; uses the sourced ROS 2 environment."""
import shlex
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    get=lambda name:LaunchConfiguration(name).perform(context)
    args=[get('mode')]
    for key in ('root','dataset','start','end','rate'):
        value=get(key)
        if value:args.extend(['--'+key,value])
    args.extend(shlex.split(get('options')))
    return [Node(package='conpr_file_player',executable='player',name='conpr_file_player',arguments=args,output='screen')]

def generate_launch_description():
    defaults={'root':'','dataset':'','mode':'gui','start':'0','end':'0','rate':'1','options':''}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=start)])
