import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch.event_handler

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

package_name = "rplidar_ros"
launch_dir = get_package_share_directory(package_name)+'/launch/'


def get_node(name):
    return launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(launch_dir + name))


def generate_launch_description():
    n1 = get_node('lidar_1.launch.py')
    n2 = get_node('lidar_2.launch.py')
    n3 = get_node('lidar_3.launch.py')

    return launch.LaunchDescription([
        n1,
        n2,
        n3
    ])