import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from rcl_interfaces.msg import Log
import time


class MonitoringNode(Node):
    def __init__(self):
        super().__init__('monitoring_node')
        self.last_cmd_vel_time = time.time()
        self.elapsed_time = 0.0
        self.previous_rate = 0.0
        self.rate_threshold_percentage = 20
        self.rate_buffer = []
        self.buffer_size = []

        self.log_pub = self.create_publisher(Log, '/rosout', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/topic',
            self.topic_callback,
            10  # adjust the queue size as needed
        )

    def topic_callback(self, msg: Twist):
        avg_rate = 0.0
        current_time = time.time()
        dt = current_time - self.last_cmd_vel_time
        self.last_cmd_vel_time = current_time
        topic_hz = 1.0 / dt if dt > 0 else 0.0
        self.elapsed_time += dt

        self.rate_buffer.append(topic_hz)

        # if len(self.rate_buffer) >= self.buffer_size:
        if self.elapsed_time >= 1.0:
            avg_rate = sum(self.rate_buffer) / len(self.rate_buffer)
            print(f"Avg_rate: {avg_rate:.2f}")

            threshold_value_up = self.previous_rate * \
                (1 + self.rate_threshold_percentage / 100)
            threshold_value_down = self.previous_rate * \
                (1 - self.rate_threshold_percentage / 100)

            if avg_rate > threshold_value_up or avg_rate < threshold_value_down:
                log_msg = Log()
                log_msg.name = self.get_name()
                # {avg_rate:.2f} Hz"
                log_msg.msg = f"cmd_vel publication rate changed from {self.previous_rate:.2f} to {avg_rate:.2f} Hz"
                print(log_msg.msg)

                self.log_pub.publish(log_msg)

            self.previous_rate = avg_rate
            self.rate_buffer = []
            self.elapsed_time = 0.0


def main(args=None):
    rclpy.init(args=args)

    monitoring_node = MonitoringNode()
    rclpy.spin(monitoring_node)
    monitoring_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
