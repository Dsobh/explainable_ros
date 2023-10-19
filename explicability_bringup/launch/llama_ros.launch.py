
from launch import LaunchDescription
from llama_bringup.utils import create_llama_launch


def generate_launch_description():

    return LaunchDescription([
        create_llama_launch(
            n_ctx=4096,
            n_batch=256,
            n_gpu_layers=15,
            n_threads=4,
            n_predict=-1,

            model_repo="TheBloke/dolphin-2.1-mistral-7B-GGUF",
            model_filename="dolphin-2.1-mistral-7b.Q4_K_M.gguf",

            # prefix="\n\n<|im_end|>user\n",
            # suffix="\n\n<|im_end|>\n<|im_start|>assistant\n",
            stop="<|im_start|>user\n",

            # file="dolphin-mistral.txt"
        )
    ])
