import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    explicability_node_cmd = Node(
        package="explicability_ros",
        executable="explicability_node",
        name="explicability_node"
    )

    llama_ros_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                "explicability_bringup"), "launch", "llama_ros.launch.py"))
    )

    ld = LaunchDescription()
    ld.add_action(explicability_node_cmd)
    ld.add_action(llama_ros_cmd)
    return ld
