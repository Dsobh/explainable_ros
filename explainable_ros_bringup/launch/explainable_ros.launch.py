import os
from launch import LaunchDescription
from launch_ros.actions import Node
from llama_bringup.utils import create_llama_launch_from_yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_directory = get_package_share_directory("explainable_ros_bringup")

    explainability_node_cmd = Node(
        package="explainable_ros",
        executable="explainability_node",
        name="explainability_node",
    )

    reranker_model = create_llama_launch_from_yaml(
        os.path.join(package_directory, "models", "jina-reranker.yaml")
    )

    embbedings_model = create_llama_launch_from_yaml(
        os.path.join(
            package_directory,
            "models",
            "bge-base-en-v1.5.yaml",
        )
    )

    base_model = create_llama_launch_from_yaml(
        os.path.join(package_directory, "models", "Qwen2.yaml")
    )

    ld = LaunchDescription()
    ld.add_action(reranker_model)
    ld.add_action(embbedings_model)
    ld.add_action(base_model)
    ld.add_action(explainability_node_cmd)

    return ld
