import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, PushRosNamespace
from llama_bringup.utils import create_llama_launch_from_yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_directory = get_package_share_directory("explainable_ros_bringup")
    print(f"[Launch] Package directory resolved: {package_directory}")

    explainability_node_cmd = Node(
        package="explainable_ros",
        executable="explainability_node",
        name="explainability_node",
        output="screen",
    )
    print("[Launch] Created explainability_node_cmd")

    # Model definitions with namespaces
    model_paths = {
        "reranker_model": os.path.join(package_directory, "models", "jina-reranker.yaml"),
        "embbedings_model": os.path.join(
            package_directory, "models", "bge-base-en-v1.5.yaml"
        ),
        "base_model": os.path.join(package_directory, "models", "Qwen2.yaml"),
    }

    model_launches = []
    delay_seconds = 0.0

    for name, path in model_paths.items():
        print(f"[Launch] Loading model '{name}' from: {path}")

        if not os.path.exists(path):
            raise FileNotFoundError(f"[ERROR] Model file not found: {path}")

        try:
            model_action = create_llama_launch_from_yaml(path)
            if model_action is None:
                raise ValueError(
                    f"[ERROR] Model '{name}' returned None from launch factory."
                )

            print(f"[Launch] Successfully loaded: {name}")

            # Encapsular en namespace y añadir delay
            model_launches.append(
                TimerAction(
                    period=delay_seconds,
                    actions=[
                        PushRosNamespace(name),
                        model_action,
                    ],
                )
            )

            delay_seconds += 2.0  # retraso progresivo entre modelos

        except Exception as e:
            raise RuntimeError(f"[ERROR] Failed to load model '{name}': {e}")

    # LaunchDescription con acciones
    ld = LaunchDescription()

    for action in model_launches:
        ld.add_action(action)

    print("[Launch] Adding explainability_node_cmd")
    ld.add_action(TimerAction(period=delay_seconds, actions=[explainability_node_cmd]))

    print("[Launch] Launch description ready.")
    return ld
    # reranker_model = create_llama_launch_from_yaml(
    #     os.path.join(package_directory, "models", "jina-reranker.yaml")
    # )

    # embbedings_model = create_llama_launch_from_yaml(
    #     os.path.join(
    #         package_directory,
    #         "models",
    #         "bge-base-en-v1.5.yaml",
    #     )
    # )

    # base_model = create_llama_launch_from_yaml(
    #     os.path.join(package_directory, "models", "Qwen2.yaml")
    # )

    # ld = LaunchDescription()
    # ld.add_action(embbedings_model)
    # ld.add_action(reranker_model)
    # ld.add_action(base_model)
    # ld.add_action(explainability_node_cmd)

    # return ld
