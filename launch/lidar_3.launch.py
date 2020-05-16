import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

package_name = "rplidar_ros"
node_name = "rplidarNode3"

def generate_launch_description():
    parameters_file_path = Path(get_package_share_directory(package_name), 'config', 'lidars.yaml')
    # print("Path:", parameters_file_path)
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=package_name,
            node_executable=node_name,
            output='screen',
            parameters=[parameters_file_path],
            # on_exit=launch.actions.Shutdown()
        ),
    ])