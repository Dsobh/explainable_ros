
import time
from typing import List

import rclpy
from simple_node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import Log
from explicability_msgs.srv import Question
from explicability_ros.task_creation_chain import TaskCreationChain

from langchain.vectorstores import Chroma
from langchain.docstore.document import Document
from llama_ros.langchain import LlamaROS
from llama_ros.langchain import LlamaROSEmbeddings


class ExplainabilityNode(Node):
    def __init__(self):
        super().__init__("explainability_node")

        self.logs_number = 0
        self.embedding_number = 0
        self.msg_queue = []

        # Model and embeddings from llama_ros
        local_llm = LlamaROS(node=self, temp=0.0)
        self.question_chain = TaskCreationChain.from_llm(local_llm)
        embeddings = LlamaROSEmbeddings(node=self)

        # Chroma Database
        self.db = Chroma(
            embedding_function=embeddings)

        self.retriever = self.db.as_retriever(search_kwargs={"k": 10})

        self.srv = self.create_service(
            Question, "question", self.question_server_callback,
            callback_group=ReentrantCallbackGroup())

        self.subscription = self.create_subscription(
            Log,
            "/rosout",
            self.listener_callback,
            1000,
            callback_group=ReentrantCallbackGroup())

        self.emb_timer = self.create_timer(
            1, self.emb_cb, callback_group=ReentrantCallbackGroup())

    def listener_callback(self, log: Log) -> None:
        self.logs_number += 1
        self.msg_queue.append(log)
        print(f"Log {self.logs_number}: {log.msg}")

    def emb_cb(self) -> None:

        if self.msg_queue:
            log = self.msg_queue.pop(0)
            start = time.process_time()
            self.db.add_texts(texts=[log.msg])
            self.embedding_number += 1
            print(
                f"Time to create embedding {self.embedding_number}: {time.process_time() - start}")

    def question_server_callback(
        self,
        request: Question.Request,
        response: Question.Response
    ) -> Question.Response:

        docuemnts: List[Document] = self.retriever.get_relevant_documents(
            request.question)

        logs = ""
        for d in docuemnts:
            logs += d.page_content + "\n"

        answer = self.question_chain.run(logs=logs, question=request.question)

        response.answer = answer
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExplainabilityNode()
    node.join_spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
