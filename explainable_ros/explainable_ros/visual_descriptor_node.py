import rclpy


from simple_node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import message_filters

from llama_ros.llama_client_node import LlamaClientNode
from llama_msgs.action import GenerateResponse


class VisualExplainabilityNode(Node):
    def __init__(self):
        super().__init__("visual_descriptor_node")

        self._action_client = self.create_action_client(
            GenerateResponse, "/llava/generate_response"
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # subscribers
        self.camera_subscriber = self.create_subscription(
            Image, "/camera/rgb/image_raw", self.camera_callback, qos_profile
        )

        self.last_image_msg = None

        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        print("Imagen publicada en topic")
        self.last_image_msg = msg

    def timer_callback(self):

        if self.last_image_msg is not None:
            print("Procesando imagen")

            llama_client = LlamaClientNode.get_instance()
            # VLM Code for Image-To-Text
            goal = GenerateResponse.Goal()
            goal.prompt = "Describe the environment of the image"
            goal.image = self.last_image_msg
            goal.sampling_config.temp = 0.0
            goal.reset = True

            llama_client.generate_response(goal, text_cb)

            self.last_image_msg = None
            # print(result.response.text)
            # self.get_logger().info(f"Camera information: {result.response.text}")


def text_cb(feedback):
    print(feedback.feedback.partial_response.text, end="", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = VisualExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
