import rclpy
import threading

from simple_node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from rcl_interfaces.msg import Log

from llama_ros.llama_client_node import LlamaClientNode
from llama_msgs.action import GenerateResponse


class VisualExplainabilityNode(Node):
    def __init__(self):
        super().__init__("visual_descriptor_node")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.camera_subscriber = self.create_subscription(
            Image,
            "/head_front_camera/rgb/image_raw",
            self.camera_callback,
            qos_profile,
        )

        self.description_publisher = self.create_publisher(
            Log, "/camera_descriptions", qos_profile
        )

        self.last_image_msg = None
        self.processing = False
        self.lock = threading.Lock()
        self.message_id = 0

        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        self.last_image_msg = msg

    def timer_callback(self):
        if self.last_image_msg is not None and not self.processing:
            if self.last_image_msg.data:
                print("Procesando imagen")
                self.processing = True
                thread = threading.Thread(target=self.process_image)
                thread.start()

    def process_image(self):
        with self.lock:
            llama_client = LlamaClientNode.get_instance()

            goal = GenerateResponse.Goal()
            goal.prompt = (
                "This image was captured by a robot's camera. "
                "Describe briefly in this exact format:\n"
                "Environment: [Indoor/Outdoor]\n"
                "Objects: [List object types present, no counts]\n"
                "People: [Present/None]\n"
                "Activity: [If obvious, else 'None']\n"
                "Do not provide details or explanations. Keep it short."
            )
            goal.image = self.last_image_msg
            goal.sampling_config.temp = 0.0
            goal.reset = True

            vlm_response, vlm_status_code = llama_client.generate_response(goal)

            if vlm_response and vlm_response.response.text:
                description = vlm_response.response.text

                self.get_logger().info(
                    f"Camera description - {vlm_response.response.text}"
                )

                now = self.get_clock().now().to_msg()

                log_msg = Log()
                log_msg.stamp = now
                log_msg.msg = f"[Camera description {self.message_id}] {description}"
                self.description_publisher.publish(log_msg)
                self.message_id += 1
            else:
                self.get_logger().warn("VLM response was empty or failed.")

            self.last_image_msg = None
            self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = VisualExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
