
import rclpy
from simple_node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from llama_msgs.action import GenerateResponse


class VisualExplainabilityNode(Node):
    def __init__(self):
        super().__init__("vexp_node")

        self._action_client = self.create_action_client(
            GenerateResponse, "/llava/generate_response")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.camera_sub = self.create_subscription(
            Image, "/camera", self.camera_callback, qos_profile)

    def camera_callback(self, data: Image):
        goal = GenerateResponse.Goal()
        goal.prompt = "Describe the center of the image"
        goal.image = data
        goal.sampling_config.temp = 0.0
        goal.reset = True

        self._action_client.wait_for_server()
        self._action_client.send_goal(goal)
        self._action_client.wait_for_result()
        result: GenerateResponse.Result = self._action_client.get_result()

        self.get_logger().info(f"{result.response.text}")


def main(args=None):
    rclpy.init(args=args)
    node = VisualExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
