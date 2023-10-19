
import sys
import rclpy
from rclpy.node import Node
from explicability_msgs.srv import Question


class ExplainabilityClientNode(Node):

    def __init__(self) -> None:
        super().__init__("explainability_client_node")

        self.client = self.create_client(Question, "question")

    def sed_request(self, question_string: str) -> Question.Response:

        req = Question.Request()
        req.question = question_string

        self.client.wait_for_service()
        self.future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    client_node = ExplainabilityClientNode()
    response = client_node.sed_request(sys.argv[1])
    print(f"Response: {response.answer}")
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
