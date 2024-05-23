import math

import rclpy
from simple_node import Node
import message_filters

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from llama_msgs.action import GenerateResponse


class VisualExplainabilityNode(Node):
    def __init__(self):
        super().__init__("vexp_node")

        self._action_client = self.create_action_client(
            GenerateResponse, "/llava/generate_response")

        self.previous_distance = float('inf')
        self.distance_threshold = 5

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # subscribers
        camera_sub = message_filters.Subscriber(
            self, Image, "/camera", qos_profile=10)
        plan_sub = message_filters.Subscriber(
            self, Path, "/plan", qos_profile=10)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (camera_sub, plan_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.obstacle_detection_callback)

    def calculate_distance(self, p1, p2):
        distance = math.sqrt((p2.x - p1.x)**2 +
                             (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
        return distance

    def obstacle_detection_callback(self, data: Image, plan: Path):
        total_distance = 0

        for i in range(len(plan.poses) - 1):
            p1 = plan.poses[i].pose.position
            p2 = plan.poses[i + 1].pose.position
            distance = self.calculate_distance(p1, p2)
            total_distance += distance

        if self.previous_distance != 0 and total_distance > self.previous_distance * self.distance_threshold:
            self.get_logger().info("Possible obstacle detected: Distance to the goal increase from {:.2f} meters to {:.2f} meters".format(
                self.previous_distance, total_distance))

            # VLM Code for Image-To-Text
            goal = GenerateResponse.Goal()
            goal.prompt = "What is in the center of the image?"
            goal.image = data
            goal.sampling_config.temp = 0.0
            goal.reset = True

            self._action_client.wait_for_server()
            self._action_client.send_goal(goal)
            self._action_client.wait_for_result()
            result: GenerateResponse.Result = self._action_client.get_result()

            self.get_logger().info(
                f"Camera Log for obstacle detection: {result.response.text}")

        self.previous_distance = total_distance


def main(args=None):
    rclpy.init(args=args)
    node = VisualExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
