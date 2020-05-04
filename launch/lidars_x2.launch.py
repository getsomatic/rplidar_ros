from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import os
import launch_ros.actions
import pathlib
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


parameters_file_name = 'lidars.yaml'

def generate_launch_description():
    parameters_file_path = str(pathlib.Path(__file__).parents[1])
    parameters_file_path += '/config/' + parameters_file_name
    print(parameters_file_path)
    return launch.LaunchDescription([launch.actions.DeclareLaunchArgument('node_prefix',
                                                                          default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
                                                                          description='Prefix for node names'),
                                     launch_ros.actions.Node(
                                         package='rplidar_ros', node_executable='rplidarNode',
                                         output='screen',
                                         parameters=[
                                             parameters_file_path
                                         ],
                                         node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'rplidarNode']),

                                     launch_ros.actions.Node(
                                         package='rplidar_ros', node_executable='rplidarNode',
                                         output='screen',
                                         parameters=[
                                             parameters_file_path
                                         ],
                                         node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'rplidarNode2'])])
