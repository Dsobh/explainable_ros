import sys

from explicability_msgs.srv import Question
import rclpy
from rclpy.node import Node

class ExplainabilityNodeClient(Node):

    def __init__(self):
        super().__init__('explainability_node_client')
        self.client = self.create_client(Question, 'question')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available. Waiting...')
        self.req = Question.Request()
        
    def sed_request(self, question_string):
        self.req.question = question_string
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    
def main(args=None):
    rclpy.init(args=args)
    
    client_node = ExplainabilityNodeClient()
    response = client_node.sed_request(sys.argv[1])
    client_node.get_logger().info(
        "Response: %s" % (response.answer)
    )
    client_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()