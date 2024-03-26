import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from datetime import datetime, timedelta


class MonitoringNode(Node):

    def __init__(self):
        super().__init__('monitoring_node')

        self.log_pub = self.create_publisher(Log, '/rosout', 10)
        self.subscription = self.create_subscription(
            Twist,
            'topic',
            self.listener_callback,
            10)

        # self.get_logger().info("Starting to monitor cmd_vel topic. Expected min publication rate: 10Hz. Expected max publication rate: 20Hz.")

        self.last_printed_second = None
        self.msg_count = 0
        self.last_msg_time = None
        self.total_msg_count = 0
        self.total_time_elapsed = timedelta(seconds=0)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.previous_rate = 0.0
        self.avg_frequency = 0.0
        self.rate_threshold_percentage = 50

    def log_builder(self, avg_rate, string, sec, nanosec):
        log_msg = Log()
        log_msg.name = self.get_name()
        log_msg.msg = f"cmd_vel publication rate {string} from {self.previous_rate:.2f} to {avg_rate:.2f} Hz"
        log_msg.stamp.sec = sec
        log_msg.stamp.nanosec = nanosec
        print(log_msg.msg)
        self.log_pub.publish(log_msg)

    def listener_callback(self, msg):
        current_time = datetime.now()
        if self.last_msg_time is not None:
            time_diff = (current_time - self.last_msg_time)
            self.total_time_elapsed += time_diff
        self.msg_count += 1
        self.last_msg_time = current_time

    def timer_callback(self):
        stamp_time = self.get_clock().now()
        stamp_sec = stamp_time.nanoseconds // 10**9
        stamp_nanosec = stamp_time.nanoseconds % 10**9
        current_time = datetime.now()

        if self.msg_count > 0:
            time_diff = (current_time - self.last_msg_time)
            self.total_time_elapsed += time_diff
            self.total_msg_count += self.msg_count

            self.avg_frequency = self.total_msg_count / \
                self.total_time_elapsed.total_seconds()

            # threshold_value_up = self.previous_rate * \
            #    (1 + self.rate_threshold_percentage / 100)
            # threshold_value_down = self.previous_rate * \
            #    (1 - self.rate_threshold_percentage / 100)

            # if self.avg_frequency > threshold_value_up and self.previous_rate > 1:
            #    self.log_builder(self.avg_frequency, "increase",
            #                     stamp_sec, stamp_nanosec)
            # if self.avg_frequency < threshold_value_down and self.previous_rate > 1:
            #    self.log_builder(self.avg_frequency, "decrease",
            #                     stamp_sec, stamp_nanosec)

        self.msg_count = 0
        self.last_msg_time = current_time
        self.previous_rate = self.avg_frequency

        # Print average rate each second
        if current_time.second % 5 == 0 and current_time.second != self.last_printed_second and self.avg_frequency != 0.0:
            log_msg = Log()
            log_msg.name = self.get_name()
            log_msg.msg = f"Average publication rate of cmd_vel: {self.avg_frequency:.2f}Hz."
            log_msg.stamp.sec = stamp_sec
            log_msg.stamp.nanosec = stamp_nanosec
            print(log_msg.msg)
            self.log_pub.publish(log_msg)
            self.last_printed_second = current_time.second


def main(args=None):
    rclpy.init(args=args)

    monitoring_node = MonitoringNode()
    rclpy.spin(monitoring_node)
    monitoring_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
