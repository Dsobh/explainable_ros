import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from llama_bringup.utils import create_llama_launch_from_yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_directory = get_package_share_directory("explainable_ros_bringup")
    print(f"[Launch] Package directory resolved: {package_directory}")

    embbedings_model = GroupAction(
        [
            PushRosNamespace("llama/embeddings"),
            create_llama_launch_from_yaml(
                os.path.join(package_directory, "models", "bge-base-en-v1.5.yaml")
            ),
        ]
    )

    reranker_model = GroupAction(
        [
            PushRosNamespace("llama/reranker"),
            create_llama_launch_from_yaml(
                os.path.join(package_directory, "models", "jina-reranker.yaml")
            ),
        ]
    )

    base_model = GroupAction(
        [
            PushRosNamespace("llama/base"),
            create_llama_launch_from_yaml(
                os.path.join(package_directory, "models", "Qwen2.yaml")
            ),
        ]
    )

    explainability_node_cmd = Node(
        package="explainable_ros",
        executable="explainability_node",
        name="explainability_node_main",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(embbedings_model)
    ld.add_action(reranker_model)
    ld.add_action(base_model)
    ld.add_action(explainability_node_cmd)

    return ld
