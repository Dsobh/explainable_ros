import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import Log
from explainable_ros_msgs.srv import Question


from langchain_chroma import Chroma

from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnablePassthrough
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate, HumanMessagePromptTemplate

from langchain.retrievers import ContextualCompressionRetriever
from langchain.schema import Document

from llama_ros.langchain import ChatLlamaROS, LlamaROSEmbeddings, LlamaROSReranker


class ExplainabilityNode(Node):
    def __init__(self):
        super().__init__("explainability_node")

        self.logs_number = 0
        self.total_time = 0
        self.embedding_number = 0
        self.msg_queue = []
        self.previous_msg = ""

        # To manage a queue of 10 most recent messages and avoid high amount of duplicated messages
        self.recent_msgs = []
        self.recent_msgs_conter = 0

        ###ROS Topics and services

        # Create subscription for /rosout topic
        self.subscription = self.create_subscription(
            Log,
            "/rosout",
            self.listener_callback,
            1000,
            callback_group=ReentrantCallbackGroup(),
        )

        # Create a ROS 2 Service to make question to de model
        self.srv = self.create_service(
            Question,
            "question",
            self.question_server_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.vector_db = Chroma(embedding_function=LlamaROSEmbeddings())

        self.retriever = self.vector_db.as_retriever(search_kwargs={"k": 20})

        # Create prompt
        prompt = ChatPromptTemplate.from_messages(
            [
                SystemMessage(
                    "You are analyzing ROS 2 logs with image descriptions from a service robot operating strictly indoors, inside an apartment. The robot moves slowly and captures an image every 3 seconds. Your task is to detect potential cyberattacks on the camera, such as image substitution or denial-of-service (DoS). Ignore normal repetition due to slow movement. Instead, flag any sudden, unexplained changes in the environment (e.g., indoor to outdoor), or inconsistent sequences suggesting tampering or falsification.\n\n"
                ),
                HumanMessagePromptTemplate.from_template(
                    "Taking into account the following logs:{context}\n\n{question}"
                ),
            ]
        )

        compressor = LlamaROSReranker(top_n=5)
        compression_retriever = ContextualCompressionRetriever(
            base_compressor=compressor, base_retriever=self.retriever
        )

        def format_docs(documents):
            logs = ""
            sortered_list = self.order_retrievals(documents)

            for l in sortered_list:
                logs += l

            return logs

        # Create the chain
        self.rag_chain = (
            {
                "context": compression_retriever | format_docs,
                "question": RunnablePassthrough(),
            }
            | prompt
            | ChatLlamaROS(temp=0.0)
            | StrOutputParser()
        )

        self.emb_timer = self.create_timer(0.001, self.emb_cb)

    def listener_callback(self, log: Log) -> None:
        self.logs_number += 1

        # For not considering llama_ros logs
        if log.name != "llama.llama_embedding_node":
            self.msg_queue.append(log)
            print(f"Log {self.logs_number}: {log.msg}")

    def emb_cb(self) -> None:

        if self.msg_queue:
            log = self.msg_queue.pop(0)
            start = time.time()

            # Eliminar solo el mensaje anterior
            if log.msg != self.previous_msg and log.name != "welcome_app_tiago":
                msg_sec = log.stamp.sec
                msg_nanosec = log.stamp.nanosec
                unix_timestamp = msg_sec + msg_nanosec / 1e9

                self.vector_db.add_texts([str(unix_timestamp) + " - " + log.msg])

                # self.vector_db.add_texts(texts=[str(unix_timestamp) + " - " + log.msg])
                self.previous_msg = log.msg
                self.embedding_number += 1

                emb_time = time.time() - start
                self.total_time += emb_time
                print(
                    f"Time to create embedding {self.embedding_number}: {emb_time} | Total time: {self.total_time}"
                )

    def order_retrievals(self, docuemnt_list):

        aux_list = []

        for d in docuemnt_list:
            aux_list.append(d.page_content + "\n")

        return sorted(aux_list)

    def question_server_callback(
        self, request: Question.Request, response: Question.Response
    ) -> Question.Response:

        question_text = str(request.question)
        answer = self.rag_chain.invoke(question_text)
        response.answer = answer
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ExplainabilityNode()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
